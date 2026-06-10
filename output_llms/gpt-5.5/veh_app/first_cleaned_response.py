import os
import math

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens




ChVector = getattr(chrono, "ChVector3d", chrono.ChVectorD)
ChVectorF = getattr(chrono, "ChVector3f", chrono.ChVectorF)
ChCoordsys = getattr(chrono, "ChCoordsysd", chrono.ChCoordsysD)
ChFrame = getattr(chrono, "ChFramed", chrono.ChFrameD)






if "CHRONO_DATA_DIR" in os.environ:
    chrono.SetChronoDataPath(os.environ["CHRONO_DATA_DIR"])

veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")




step_size = 1.0e-3
tire_step_size = 1.0e-3

render_fps = 60
render_step = 1.0 / render_fps
render_steps = int(math.ceil(render_step / step_size))

sensor_update_rate = 30.0
camera_width = 1280
camera_height = 720
camera_fov = 1.408  

end_time = 120.0


init_loc = ChVector(0.0, 0.0, 0.45)
init_rot = chrono.QUNIT




gator = veh.Gator()

gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(ChCoordsys(init_loc, init_rot))


gator.SetChassisCollisionType(veh.CollisionType_NONE)


try:
    gator.SetTireType(veh.TireModelType_RIGID)
except Exception:
    pass

try:
    gator.SetTireStepSize(tire_step_size)
except Exception:
    pass


gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

gator.Initialize()

system = gator.GetSystem()
system.SetGravitationalAcceleration(ChVector(0.0, 0.0, -9.81))




terrain = veh.RigidTerrain(system)

try:
    terrain_mat = chrono.ChContactMaterialNSC()
except Exception:
    terrain_mat = chrono.ChMaterialSurfaceNSC()

terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    terrain_mat,
    ChCoordsys(ChVector(0.0, 0.0, 0.0), chrono.QUNIT),
    200.0,
    200.0,
)

patch.SetColor(chrono.ChColor(0.35, 0.45, 0.25))

try:
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
except Exception:
    pass

terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono Gator Vehicle with Sensors")
vis.SetWindowSize(1280, 720)


vis.SetChaseCamera(ChVector(0.0, 0.0, 0.75), 6.0, 0.5)

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(gator.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step / steering_time)
driver.SetThrottleDelta(render_step / throttle_time)
driver.SetBrakingDelta(render_step / braking_time)

driver.Initialize()

print("Interactive driver controls:")
print("  W / Up Arrow    : throttle")
print("  S / Down Arrow  : brake")
print("  A / Left Arrow  : steer left")
print("  D / Right Arrow : steer right")




manager = sens.ChSensorManager(system)


manager.scene.AddPointLight(
    ChVectorF(8.0, -6.0, 8.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)

manager.scene.AddPointLight(
    ChVectorF(-8.0, 6.0, 6.0),
    chrono.ChColor(0.6, 0.7, 1.0),
    300.0,
)


try:
    chassis_body = gator.GetChassisBody()
except Exception:
    chassis_body = gator.GetVehicle().GetChassisBody()




camera_offset_pose = ChFrame(
    ChVector(0.6, 0.0, 1.25),
    chrono.QUNIT,
)

camera = sens.ChCameraSensor(
    chassis_body,
    sensor_update_rate,
    camera_offset_pose,
    camera_width,
    camera_height,
    camera_fov,
)

camera.SetName("Gator chassis camera")


camera.PushFilter(
    sens.ChFilterVisualize(
        camera_width,
        camera_height,
        "Gator Chassis Camera",
    )
)


camera.PushFilter(sens.ChFilterRGBA8Access())




manager.AddSensor(camera)




realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = system.GetChTime()

    if time >= end_time:
        break

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    
    manager.Update()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    step_number += 1