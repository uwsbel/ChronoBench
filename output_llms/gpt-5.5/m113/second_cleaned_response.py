import math

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh





chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")





initLoc = chrono.ChVector3d(-15, 0, 0.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)





vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

trackPoint = chrono.ChVector3d(0.0, 0.0, 0.1)

contact_method = chrono.ChContactMethod_SMC
contact_vis = False





step_size = 5e-4
render_step_size = 1.0 / 50.0

render_steps = math.ceil(render_step_size / step_size)





terrainHeight = 0.0
terrainLength = 100.0
terrainWidth = 100.0
terrain_delta = 0.04

height_map_file = veh.GetDataFile("terrain/height_maps/bump64.bmp")
height_map_min = -0.05
height_map_max = 0.05





vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)





terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())

terrain.SetPlane(
    chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, terrainHeight),
        chrono.QUNIT
    )
)






terrain.SetSoilParameters(
    0.2e6,   
    0.0,     
    1.1,     
    0.0,     
    30.0,    
    0.01,    
    2.0e8,   
    3.0e4    
)

if contact_vis:
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0.0, 0.1)

terrain.Initialize(
    height_map_file,
    terrainLength,
    terrainWidth,
    height_map_min,
    height_map_max,
    terrain_delta
)

terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 20)





vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 SCM Terrain Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
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





print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())





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