import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-15, 0, 0.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 0.1)


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

vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.DeformableTerrainSCM(vehicle.GetSystem())
terrain.SetPlane(chrono.ChVector3d(0, 0, 1), terrainHeight)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
terrain.SetSCMContactFriction(0.9)
terrain.SetSCMDampingF(0.2)
terrain.SetSCMDampingD(0.02)
terrain.SetSCMCohesion(5000)
terrain.SetSCMFriction(0.5)
terrain.SetSCMCriticalFriction(0.8)
terrain.SetSCMFrictionTension(1000)
terrain.SetSCMFrictionCompression(3000)
terrain.SetSCMFrictionViscosity(20)
terrain.SetSCMFrictionRelaxation(0.5)
terrain.SetSCMFrictionAngle(0.5)
terrain.SetSCMFrictionDecay(0.5)
terrain.SetSCMFrictionExpansion(0.5)
terrain.SetSCMFrictionContraction(0.5)
terrain.SetSCMFrictionRepose(0.5)
terrain.SetSCMFrictionRolling(0.5)
terrain.SetSCMFrictionSpinning(0.5)
terrain.SetSCMFrictionSliding(0.5)
terrain.SetSCMFrictionSlidingViscosity(0.5)
terrain.SetSCMFrictionSlidingRelaxation(0.5)
terrain.SetSCMFrictionSlidingExpansion(0.5)
terrain.SetSCMFrictionSlidingContraction(0.5)
terrain.SetSCMFrictionSlidingRepose(0.5)
terrain.SetSCMFrictionSlidingRolling(0.5)
terrain.SetSCMFrictionSlidingSpinning(0.5)
terrain.SetSCMFrictionSlidingSliding(0.5)
terrain.Initialize()



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



vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


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
    driver_inputs.m_throttle = 0.8
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1