"""HMMWV ROS vehicle demo using an NSC system, rigid textured terrain, and Irrlicht.

The simulation builds a full HMMWV wrapper vehicle, publishes simulation clock,
chassis body state, and driver input subscriptions through ChROS, then renders the
vehicle driving on a flat terrain patch with mesh/primitives visualization.
"""

import math

import pychrono.core as chrono
from pychrono import irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.ros as chros


# === Parameters === named constants keep the vehicle setup and loop reproducible
step_size = 2.0e-3
tire_step_size = step_size
sim_end = 6.0
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_steps = max(1, math.ceil(render_step_size / step_size))  # precomputed once
terrain_length = 100.0
terrain_width = 100.0
init_loc = chrono.ChVector3d(0.0, 0.0, 0.6)
init_rot = chrono.QUNIT


# === Vehicle and system === wrapper creates bodies, joints, tires, and owned ChSystemNSC
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemNSC reused every step
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: chassis body reused by ROS/logging
veh_obj = vehicle.GetVehicle()  # cache: underlying vehicle reused by visualization/ROS
chassis.SetName("hmmwv_chassis")
# wheels/spindles, suspension, steering, driveline, and tire bodies are created inside HMMWV_Full.
print("VEHICLE MASS: ", veh_obj.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === rigid ground patch provides stable vehicle contact and visible texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Visualization and driver === Irrlicht vehicle visual system mirrors catalog demos
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV ROS")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_obj)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


# === ROS bridge === publishes clock/chassis and subscribes to vehicle driver commands
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25.0, driver, "~/input/driver_inputs"))
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25.0, chassis, "~/output/hmmwv_chassis"))
ros_manager.Initialize()


# === Main loop === render at fixed cadence and advance the full vehicle subsystem stack
frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

try:

    while vis.Run() and system.GetChTime() < sim_end:
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frame += 1

        time = system.GetChTime()
        driver_inputs = driver.GetInputs()  # cache: passed to vehicle and visual systems

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        if not ros_manager.Update(time, step_size):
            break


        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid Chrono state
    raise
except (OSError, IOError) as exc:  # disk or permission failures while recording
    raise
finally:
    pass


# === Post-processing === review video and tables are stripped from accepted source
