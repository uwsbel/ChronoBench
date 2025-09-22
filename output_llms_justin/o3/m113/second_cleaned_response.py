import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')




init_loc = chrono.ChVector3d(-15.0, 0.0, 0.0)          
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)




vis_type = veh.VisualizationType_MESH
contact_method = chrono.ChContactMethod_SMC
step_size = 5.0e-4
render_step_size = 1.0 / 50.0          
terrain_length = 100.0
terrain_width  = 100.0




vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())


terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))



Bekker_Kphi = 2.1e6      
Bekker_Kc   = 0.0         
Bekker_n    = 1.1         
cohesion    = 1.3e4       
friction    = 30.0        
shear       = 0.01        
terrain.SetSoilParameters(Bekker_Kphi, Bekker_Kc, Bekker_n,
                          cohesion, friction, shear)


terrain.SetElasticStiffness(2.0e8)
terrain.SetDamping(3.0e4)


height_map_file = chrono.GetChronoDataFile('vehicle/terrain/height_maps/bump64.png')

terrain.Initialize(height_map_file, terrain_length, terrain_width, 0.0, 0.02)


terrain.SetTexture(chrono.GetChronoDataFile('vehicle/terrain/textures/dirt.jpg'),
                   16.0, 16.0)




vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 – SCM deformable terrain demo')
vis.SetWindowSize(1280, 1024)
track_point = chrono.ChVector3d(0.0, 0.0, 0.1)
vis.SetChaseCamera(track_point, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0     
throttle_time = 1.0     
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()




vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)


print("VEHICLE MASS:", vehicle.GetVehicle().GetMass(), "kg")




render_steps   = math.ceil(render_step_size / step_size)
step_number    = 0
render_frame   = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    drv_in = driver.GetInputs()
    drv_in.m_throttle = 0.8      
    

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, drv_in)
    vis.Synchronize(time, drv_in)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1