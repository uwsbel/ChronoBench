"""HMMWV vehicle application with an onboard depth camera.

The simulation uses the HMMWV_Full catalog vehicle on NSC rigid terrain, an
Irrlicht vehicle visualizer, and an OptiX depth camera attached to the chassis.
The vehicle state is logged at every dynamics step with position and heading.
"""

import csv
import math
import traceback

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Parameters ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

step_size = 1.0e-3
tire_step_size = step_size
sim_end = 1.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once

terrain_length = 100.0
terrain_width = 100.0
init_loc = chrono.ChVector3d(0.0, 0.0, 0.6)
init_rot = chrono.QUNIT

depth_width = 1280
depth_height = 720
depth_fov = 1.408
depth_max = 30.0
depth_rate = 30.0
depth_offset = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0.0, 2.0),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0.0, 1.0, 0.0)),
)


# === Vehicle And Terrain ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemNSC reused by terrain and sensors
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = hmmwv.GetChassisBody()  # cache: chassis body reused for camera and logging
vehicle_model = hmmwv.GetVehicle()  # cache: vehicle subsystem handle reused for diagnostics
# Wrapper-created essentials: chassis, suspension, steering, wheels, tires, and joints are owned by HMMWV_Full.
print("VEHICLE MASS: ", vehicle_model.GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Depth Camera Sensor ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 100.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)
manager.scene.AddAreaLight(
    chrono.ChVector3f(0.0, 0.0, 4.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
    chrono.ChVector3f(1.0, 0.0, 0.0),
    chrono.ChVector3f(0.0, -1.0, 0.0),
)

depth_camera = sens.ChDepthCamera(
    chassis,
    depth_rate,
    depth_offset,
    depth_width,
    depth_height,
    depth_fov,
    depth_max,
)
depth_camera.SetName("Depth Camera")
depth_camera.SetLag(0.0)
depth_camera.SetCollectionWindow(0.0)
depth_camera.PushFilter(sens.ChFilterDepthAccess())
depth_camera.PushFilter(sens.ChFilterDepthToRGBA8())
depth_camera.PushFilter(sens.ChFilterVisualize(depth_width, depth_height, "Depth Map"))
depth_camera.PushFilter(sens.ChFilterSave("cam/depth/"))
depth_camera.PushFilter(sens.ChFilterRGBA8Access())
manager.AddSensor(depth_camera)


# === Visualization And Driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Depth Camera")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_model)

driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta((1.0 / render_fps) / steering_time)
driver.SetThrottleDelta((1.0 / render_fps) / throttle_time)
driver.SetBrakingDelta((1.0 / render_fps) / braking_time)
driver.Initialize()


# === Main Loop ===
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    with open("vehicle_state_log.csv", "w", newline="") as state_file:
        state_writer = csv.writer(state_file)
        state_writer.writerow(["time", "x", "y", "z", "heading"])

        while vis.Run() and system.GetChTime() < sim_end:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(render_every):
                time = system.GetChTime()
                driver.Synchronize(time)
                driver_inputs = driver.GetInputs()

                terrain.Synchronize(time)
                hmmwv.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                driver.Advance(step_size)
                terrain.Advance(step_size)
                hmmwv.Advance(step_size)
                vis.Advance(step_size)
                manager.Update()

                pos = chassis.GetPos()
                heading = chassis.GetRot().GetCardanAnglesXYZ().z
                state_writer.writerow([time, pos.x, pos.y, pos.z, heading])

                depth_buffer = depth_camera.GetMostRecentDepthBuffer()
                if depth_buffer.HasData():
                    pass  # guard: the depth stream has produced at least one frame

                if system.GetChTime() >= sim_end:
                    break

                realtime_timer.Spin(step_size)
except (OSError, IOError) as exc:
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
