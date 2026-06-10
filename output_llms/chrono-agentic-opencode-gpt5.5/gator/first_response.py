"""Gator vehicle on flat rigid terrain with NSC contact and Irrlicht.

The simulation builds a catalog Gator utility vehicle, a textured rigid terrain
patch, mesh visualization for the vehicle components, and an interactive
Irrlicht driver. The vehicle should drive over the flat terrain in real time
with steering, throttle, and braking controlled through the Irrlicht driver.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Parameters === fixed demo constants and precomputed render cadence
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

step_size = 1e-3
tire_step_size = step_size
sim_end = 8.0
render_fps = 50.0
render_step = 1.0 / render_fps
render_steps = math.ceil(render_step / step_size)  # precomputed once for 50 FPS rendering

terrain_length = 100.0
terrain_width = 100.0
terrain_texture_u = 200.0
terrain_texture_v = 200.0
init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.QUNIT


# === Vehicle === wrapper-owned system with TMEASY tires and visible mesh parts
vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)  # prompt: TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemNSC reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: main chassis body reused for logging
veh_obj = vehicle.GetVehicle()  # cache: underlying wheeled vehicle handle reused below
# wrapper-created bodies include chassis, suspension, steering, wheels, and tires;
# wrapper-created joints constrain suspension and steering links to the chassis.
print("VEHICLE MASS: ", veh_obj.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === rigid flat support patch with a custom bundled texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), terrain_texture_u, terrain_texture_v)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

spindle_positions = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - 0.30
assert wheel_bottom_z > -0.10, f"Gator wheel bottoms start below terrain: z={wheel_bottom_z:.3f}"


# === Visualization === vehicle-aware Irrlicht window with chase camera and light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_obj)


# === Driver === interactive keyboard driver for steering, throttle, and braking
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step / 1.0)
driver.SetThrottleDelta(render_step / 1.0)
driver.SetBrakingDelta(render_step / 0.3)
driver.Initialize()


# === Main loop === real-time synchronization and 50 FPS rendering
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:

    while vis.Run() and system.GetChTime() < sim_end:
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        time = system.GetChTime()
        driver_inputs = driver.GetInputs()  # cache: one input struct for all subsystem sync calls
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (OSError, IOError) as exc:  # operating-system failures during runtime
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # Chrono runtime failure or invalid state
    traceback.print_exc()
    raise
finally:
    print("Simulation loop ended cleanly or after a reported runtime error.")
