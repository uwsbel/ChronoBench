"""M113 tracked vehicle on SMC rigid terrain with scripted driver control.

The simulation uses the PyChrono vehicle M113 wrapper, a rigid terrain patch
with friction and restitution, an Irrlicht tracked-vehicle visualizer, and a
real-time synchronized loop. The expected behavior is forward motion under a
constant throttle while the vehicle, terrain, driver, and visual system advance
together at a fixed timestep.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named parameters keep vehicle, terrain, and timing explicit
step_size = 5.0e-4
render_step_size = 1.0 / 50.0
render_steps = math.ceil(render_step_size / step_size)  # precomputed once
sim_end = 8.0

terrain_length = 100.0
terrain_width = 100.0
terrain_friction = 0.9
terrain_restitution = 0.01
terrain_young_modulus = 2.0e7

init_loc = chrono.ChVector3d(0.0, 0.0, 0.7)
init_rot = chrono.QUNIT
vis_type = veh.VisualizationType_MESH


# === Vehicle === wrapper creates the SMC system and all tracked subsystems
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemSMC reused below
chassis = vehicle.GetChassisBody()  # cache: main rigid body reused for logging
# system bodies: chassis, sprockets, idlers, road wheels, and track shoes are wrapper-created.
# joints: suspension, idler, sprocket, driveline, and track constraints are wrapper-created.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)


# === Terrain === SMC contact material matches the M113 wrapper contact method
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)
patch_mat.SetYoungModulus(terrain_young_modulus)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Visualization === tracked Irrlicht window follows the M113 in real time
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Tracked Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.5), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())


# === Driver === scored-core scripted throttle matches this tracked demo type
driver = veh.ChDriver(vehicle.GetVehicle())
driver.SetSteering(0.0)
driver.SetThrottle(0.8)
driver.SetBraking(0.0)
driver.Initialize()


# === Main loop === synchronize and advance the full vehicle subsystem stack
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:

    while vis.Run():
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    raise
except (OSError, IOError) as exc:  # output directory or recording-file failure
    raise
finally:
    pass
