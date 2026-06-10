import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import math





initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


terrainHeight = 0
terrainLength = 200.0
terrainWidth = 200.0


point_light_intensity = 2.0


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


t_end = 1000


cam_offset = chrono.ChVector3d(-8, 0, 3)




gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.Initialize()


gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)




terrain = veh.RigidTerrain(gator.GetSystem())

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0   
throttle_time = 1.0   
braking_time = 0.3    

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()




manager = sens.ChSensorManager(gator.GetSystem())


manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(1, 1, 1),
    500.0
)


light_positions = [
    chrono.ChVector3f(50, 50, 10),
    chrono.ChVector3f(-50, 50, 10),
    chrono.ChVector3f(50, -50, 10),
    chrono.ChVector3f(-50, -50, 10),
]

for pos in light_positions:
    manager.scene.AddPointLight(
        pos,
        chrono.ChColor(1, 1, 1),
        point_light_intensity * 100
    )





image_width = 1280
image_height = 720
fov = math.pi / 3  


cam_update_rate = 30


offset_pose = chrono.ChFramed(
    cam_offset,
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)


cam = sens.ChCameraSensor(
    gator.GetChassisBody(),  
    cam_update_rate,         
    offset_pose,             
    image_width,             
    image_height,            
    fov                      
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)


cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera View"))
cam.PushFilter(sens.ChFilterRGBA8Access())


manager.AddSensor(cam)




render_steps = math.ceil(render_step_size / step_size)




realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
time = 0

print("Starting Gator vehicle simulation...")
print("Use arrow keys to control the vehicle:")
print("  UP/DOWN: throttle/brake")
print("  LEFT/RIGHT: steering")

while vis.Run():
    time = gator.GetSystem().GetChTime()

    
    if time >= t_end:
        break

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    
    manager.Update()

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

print("Simulation completed!")