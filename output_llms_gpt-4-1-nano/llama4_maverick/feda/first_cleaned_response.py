import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(0, 0, 1.0)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type =  veh.VisualizationType_MESH
steering_vis_type =  veh.VisualizationType_MESH
wheel_vis_type =  veh.VisualizationType_MESH
tire_vis_type =  veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrain_height = 0;      
terrain_length = 100.0;  
terrain_width = 100.0;   


trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 2e-3


render_step_size = 1.0 / 50  



print( "Copyright (c) 2023 projectchrono.org\n")






my_feda = veh.FEDA(veh.VehicleSide_RIGHT, contact_method, tire_model)
my_feda.SetContactMethod(contact_method)
my_feda.SetChassisCollisionType(chassis_collision_type)
my_feda.SetChassisVisType(chassis_vis_type)
my_feda.SetSuspensionVisType(suspension_vis_type)
my_feda.SetSteeringVisType(steering_vis_type)
my_feda.SetWheelVisType(wheel_vis_type)
my_feda.SetTireVisType(tire_vis_type)
my_feda.Initialize(chrono.ChCoordsysD(initLoc, initRot))
my_feda.SetTireStepSize(step_size)
my_feda.SetAerodynamicDrag(0.5, 5.0, 1.2)


terrain = veh.RigidTerrain(my_feda.GetSystem())
patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_feda.GetSystem())
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('FEDA Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, -12, 2), trackPoint)
vis.AddTypicalLights()


driver = veh.ChIrrGuiDriver(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


render_step_size = 1.0 / 50
render_steps = int(render_step_size / step_size)
step_number = 0
render_number = 0

realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run() :
    time = my_feda.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    my_feda.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize("", driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    my_feda.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)


vis.GetDevice().closeDevice()
print("FEDA Demo terminated successfully.")