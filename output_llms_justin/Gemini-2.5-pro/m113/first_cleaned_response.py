import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import os
import math






try:
    chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
    print(f"Using CHRONO_DATA_DIR: {os.environ['CHRONO_DATA_DIR']}")
except KeyError:
    
    
    print("CHRONO_DATA_DIR environment variable not set.")
    default_data_path = os.path.join(os.path.dirname(chrono.__file__), '..', '..', '..', 'data', '')
    chrono.SetChronoDataPath(default_data_path)
    print(f"Attempting to use default data path: {os.path.abspath(default_data_path)}")


vehicle.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
print(f"Vehicle data path: {vehicle.GetDataPath()}")





step_size = 2e-3  


init_loc = chrono.ChVectorD(0, 0.7, 0)  
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  


terrain_height = 0.0
terrain_size_x = 200.0  
terrain_size_y = 200.0  
terrain_friction = 0.9
terrain_restitution = 0.01
terrain_young_modulus = 2e7  
terrain_poisson_ratio = 0.3


camera_chase_track_point = chrono.ChVectorD(0.0, 0.0, 0.0) 
camera_chase_distance = 8.0 
camera_chase_height = 1.5   







my_system = chrono.ChSystemNSC()
my_system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))


my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) 
my_system.SetSolverMaxIterations(150)
my_system.SetMaxPenetrationRecoverySpeed(4.0)














my_m113 = vehicle.M113_Vehicle(
    fixed=False,
    driveline_type=vehicle.DrivelineTypeWV.SIMPLE,
    brake_type=vehicle.BrakeType.SIMPLE,
    engine_model=vehicle.EngineModelType.SIMPLE_MAP,
    transmission_model=vehicle.TransmissionModelType.AUTOMATIC_SIMPLE_MAP,
    system=my_system,
    chassis_collision_type=vehicle.ChassisCollisionType.NONE
)


my_m113.Initialize(chrono.ChCoordsysD(init_loc, init_rot))


my_m113.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
my_m113.SetSuspensionVisualizationType(vehicle.VisualizationType_PRIMITIVES)
my_m113.SetSteeringVisualizationType(vehicle.VisualizationType_PRIMITIVES)
my_m113.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
my_m113.SetTrackShoeVisualizationType(vehicle.VisualizationType_MESH)



terrain = vehicle.RigidTerrain(my_system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)










terrain.AddPatch(patch_mat,
                 chrono.CSYSNORM, 
                 terrain_size_x, terrain_size_y)




terrain.Initialize()


terrain_asset = terrain.GetGroundBody().GetAssets()[0]
visual_asset = chrono.CastToChVisualization(terrain_asset)
visual_asset.material_list[0].SetKdTexture(chrono.GetChronoDataPath() + "textures/concrete.jpg")















vis_app = vehicle.ChVehicleIrrApp(my_m113, "M113 Simulation")
vis_app.SetSkyBox()
vis_app.AddTypicalLights(irr.vector3df(30, -30, 100), irr.vector3df(30, 50, 100), 250, 130)
vis_app.SetChaseCamera(camera_chase_track_point, camera_chase_distance, camera_chase_height)
vis_app.SetTimestep(step_size) 



driver = vehicle.ChIrrGuiDriver(vis_app)



driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)


my_m113.SetDriver(driver) 


vis_app.AssetBindAll()
vis_app.AssetUpdateAll()





rt_timer = chrono.ChRealtimeStepTimer()

print("Simulation started. Close the Irrlicht window to end.")

while vis_app.Run():
    time = my_system.GetChTime()

    
    vis_app.BeginScene(True, True, irr.SColor(255, 140, 160, 190))
    vis_app.DrawAll() 
    
    
    
    
    
    
    driver_inputs = driver.GetInputs()

    
    
    
    driver.Synchronize(time)
    my_m113.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time) 
    vis_app.Synchronize(my_m113.GetReportContactRecord()) 

    
    my_m113.Advance(step_size)
    terrain.Advance(step_size) 
    vis_app.Advance(step_size) 
    my_system.DoStepDynamics(step_size) 

    vis_app.EndScene()

    
    rt_timer.Spin(step_size)

print("Simulation ended.")
del vis_app 
del my_system