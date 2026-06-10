import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type    = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type   = veh.VisualizationType_PRIMITIVES
wheel_vis_type      = veh.VisualizationType_MESH
tire_vis_type       = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 100.0   
terrainWidth  = 100.0   


contact_method = chrono.ChContactMethod_NSC


step_size = 2e-3
tire_step_size = 1e-3





gator = veh.Gator()
gator.SetContactMethod(contact_method)
gator.SetChassisCollisionType(chassis_collision_type)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(tire_model)
gator.SetTireStepSize(tire_step_size)


gator.Initialize()


gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)


system = gator.GetSystem()





terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()





driver = veh.ChInteractiveDriverIRR  

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AttachVehicle(gator.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()





manager = sens.ChSensorManager(system)


intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(100, 100, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0)
manager.scene.AddPointLight(
    chrono.ChVector3f(-100, -100, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0)


update_rate = 30          
image_width = 1280
image_height = 720
fov = 1.408               
lag = 0.0
exposure_time = 0.0


offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0.0, 2.0),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))


cam = sens.ChCameraSensor(
    gator.GetChassisBody(),   
    update_rate,              
    offset_pose,              
    image_width,              
    image_height,             
    fov                       
)
cam.SetName("Chassis Camera")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)


cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())


manager.AddSensor(cam)






gator.GetVehicle().EnableRealtime(True)

render_step_size = 1.0 / 50  
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

print("Starting Gator vehicle simulation...")

while vis.Run():
    time = system.GetChTime()

    
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

    
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

print("Simulation finished.")