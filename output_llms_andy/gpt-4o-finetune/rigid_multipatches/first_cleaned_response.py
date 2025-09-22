import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY



terrain_height = 0      
terrain_length = 100.0  
terrain_width = 100.0   


trackpoint = chrono.ChVector3d(-15.0, 15.0, 5.8)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  




vehicle = veh.HMMWV_Full() 
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)


vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch1 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1.5), chrono.QUNIT),
    100, 100, 
    veh.GetDataFile("terrain/textures/tile4.jpg"), True, 10, 10)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch1.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 6, 6)

patch2 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(-40, 0, -1.5)), chrono.QuatFromAngleAxis(30, chrono.ChVector3d(0, 1, 0).GetWPtr()),
    20, 100, 
    veh.GetDataFile("terrain/textures/grass.jpg"), True, 6, 6);
patch2.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

patch3 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(60, 0, -1.5)), chrono.QuatFromAngleAxis(-25, chrono.ChVector3d(0, 1, 0).GetWPtr()),
    chrono.GetChronoDataFile("vehicle/terrain/mesh/isoRough2.bin"), 
    veh.GetDataFile("terrain/textures/dirt.jpg"), True, 6, 6)

patch4 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(-60, 40, -1), chrono.QUNIT),
    chrono.GetChronoDataFile("vehicle/terrain/height_maps/bump64.bmp"),
    64.0, 64.0, 0.0, 3.0, 
    veh.GetDataFile("terrain/textures/bluegray.png"), True, 12, 12)

terrain.AddFEMBrick(chrono.GetChronoDataFile("vehicle/terrain/fem/mesh_corotate.in"), 10e6, 0.3)

terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Rigs of rods')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(trackpoint, 6.0, 0.5)
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
driver.SetThrottlingDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_PSOR)
solver = vehicle.GetSystem().GetSolver()
solver.SetMaxIterations(40)


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.floor(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

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
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)