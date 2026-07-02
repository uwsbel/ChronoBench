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


soil = veh.SCMSoilParameters()
soil.SetCohesion(0.0)
soil.SetFriction(0.4)
soil.SetYoungModulus(2e6)
soil.SetPoissonRatio(0.3)
soil.SetDensity(2000)

terrain = veh.SCMDeformableTerrain(vehicle.GetSystem(), soil)
terrain.SetHeightMap(veh.GetDataFile("terrain/heightmaps/hmapBig30.png"), 0.001, -50, -50, 50, 50, 100, 100)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
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


vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0
vehicle.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = 0.0
    driver_inputs.m_throttle = 0.8
    driver_inputs.m_braking = 0.0

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1