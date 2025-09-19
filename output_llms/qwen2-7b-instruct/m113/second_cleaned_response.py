import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.scm as scm
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
scm.SetDataPath(chrono.GetChronoDataPath() + 'scm/')


initLoc = chrono.ChVectorD(-15, 0, 0.0)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


terrain_height_map = chrono.ChDataFile('terrain_height_map.txt')
terrain = scm.ChSCMDeformableTerrain(vehicle.GetSystem())
terrain.Initialize(terrain_height_map)


soil_parameters = chrono.ChSCMSoilParams()
soil_parameters.mu = 0.5
soil_parameters.lambda_ = 0.4
soil_parameters.alpha = 0.3
soil_parameters.beta = 0.2
soil_parameters.gamma = 0.1
soil_parameters.k_s = 10000
soil_parameters.k_d = 1000
soil_parameters.k_t = 100
soil_parameters.k_r = 100
soil_parameters.k_s = 10000
soil_parameters.k_d = 1000
soil_parameters.k_t = 100
soil_parameters.k_r = 100
terrain.SetSoilParams(soil_parameters)


trackPoint = chrono.ChVectorD(0.0, 0.0, 0.1)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 5e-4
tire_step_size = step_size


render_step_size = 1.0 / 50  



vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain_model = scm.ChSCMDeformableTerrain(vehicle.GetSystem())
terrain_model.SetHeightMap(terrain_height_map)
terrain_model.SetSoilParams(soil_parameters)
terrain_model.Initialize()



vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


throttle_value = 0.8



vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0
vehicle.GetVehicle().EnableRealtime(True)
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()
    
    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
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
    
    vehicle.GetVehicle().GetEngine().SetThrottle(throttle_value)