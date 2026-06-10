"""PyChrono Kraz tractor-trailer on rigid terrain.

This NSC vehicle simulation uses the catalog Kraz wrapper, a flat rigid terrain
patch with friction and restitution, an Irrlicht interactive driver, and the
standard real-time vehicle synchronization loop. The expected behavior is a
Kraz truck initialized on the terrain and controlled through the driver system
while the Irrlicht chase camera follows the tractor.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === define vehicle, terrain, and visualization timing once
step_size = 1e-3
tire_step_size = step_size
sim_end = 6.0
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_steps = max(1, math.ceil(render_step_size / step_size))  # precomputed once

terrain_length = 200.0
terrain_width = 200.0
terrain_friction = 0.9
terrain_restitution = 0.01
init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.QUNIT
init_fwd_vel = 0.0


# === Vehicle system === catalog wrapper owns the ChSystem and main bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.Kraz()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetInitFwdVel(init_fwd_vel)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystem reused throughout
tractor = vehicle.GetTractor()  # cache: visualizer attaches to the tractor vehicle
chassis = vehicle.GetTractorChassisBody()  # cache: tractor chassis handle for state access
trailer = vehicle.GetTrailer()  # cache: trailer handle proves full Kraz rig is present
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", tractor.GetMass())

# wrapper-created essentials: tractor, trailer, chassis, suspension, steering,
# wheels, tires, and joints are created by veh.Kraz(); terrain is added below.

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)


# === Terrain === rigid NSC patch supplies contact, friction, and restitution
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Visualization and driver === Irrlicht vehicle view plus interactive controls
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz rigid terrain")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(tractor)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


# === Main loop === synchronize and advance every vehicle subsystem in order
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        for _ in range(render_steps):
            time = system.GetChTime()
            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()  # cache: one input snapshot for all sync calls

            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(step_size)
            terrain.Advance(step_size)
            vehicle.Advance(step_size)
            vis.Advance(step_size)

            step_number += 1
            realtime_timer.Spin(step_size)
            if system.GetChTime() >= sim_end:
                break
except (OSError, IOError) as exc:  # file output path or permission failure
    print(f"review output failed: {exc}")
    raise
except (RuntimeError, ValueError) as exc:  # solver or invalid-state failure
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
