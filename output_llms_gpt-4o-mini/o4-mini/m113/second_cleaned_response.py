import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")





initLoc = chrono.ChVector3d(-15.0, 0.0, 0.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH

chassis_collision_type = veh.CollisionType_NONE


trackPoint = chrono.ChVector3d(0.0, 0.0, 0.1)


contact_method = chrono.ChContactMethod_SMC


step_size = 5e-4
render_step_size = 1.0 / 50  




vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(
    veh.TransmissionModelType_AUTOMATIC_SHAFTS
)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)


vehicle.SetChassisCollisionType(chassis_collision_type)


vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(
    chrono.ChCollisionSystem.Type_BULLET
)




terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())



terrain.SetSoilParameters(
    2e6,       
    0.0,       
    1e7,       
    0.3,       
    2e5,       
    2e3,       
    2e-1,      
    0.0        
)


height_map_file = chrono.GetChronoDataFile("terrain/height_maps/heightmap.png")


terrain.Initialize(height_map_file, 100.0, 100.0, 0.01)


terrain.SetTexture(
    chrono.GetChronoDataFile("terrain/textures/dirt.jpg"), 200, 200
)




vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 SCM Terrain Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)  




driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)   
driver.SetThrottleDelta(render_step_size / 1.0)   
driver.SetBrakingDelta(render_step_size / 0.3)    
driver.Initialize()




vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)


print("VEHICLE MASS: ", vehicle.GetMass())


render_steps = math.ceil(render_step_size / step_size)




step_number = 0
vis.EnableRealtime(True)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    inputs = driver.GetInputs()  
    inputs.m_throttle = 0.8

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, inputs)
    vis.Synchronize(time, inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1