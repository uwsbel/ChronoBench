import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath('./chrono_data/')
veh.SetDataPath('./chrono_data/vehicle/')


initLoc = chrono.ChVector3d(0, 0, 1.0)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


contact_method = chrono.ChContactMethod_SMC  


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.RigidTerrain(vehicle.GetSystem())


patch_mat = chrono.ChContactMaterialSMC()  
if contact_method == chrono.ChContactMethod_NSC:
    patch_mat = chrono.ChContactMaterialNSC()


patch1_mat = patch_mat
patch1 = terrain.AddPatch(patch1_mat, 
                          chrono.ChCoordsysd(chrono.ChVector3d(-50, -50, 0), chrono.QUNIT), 
                          100, 100)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


patch2_mat = patch_mat
patch2 = terrain.AddPatch(patch2_mat, 
                          chrono.ChCoordsysd(chrono.ChVector3d(50, -50, 0), chrono.QUNIT), 
                          veh.GetDataFile("terrain/height_maps/test64.bmp"), 128, 128, 0, 4)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)
patch2.SetColor(chrono.ChColor(0.5, 0.8, 0.5))


patch3_mat = patch_mat
patch3 = terrain.AddPatch(patch3_mat, 
                          chrono.ChCoordsysd(chrono.ChVector3d(-50, 50, 0), chrono.QUNIT), 
                          100, 100)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch3.SetColor(chrono.ChColor(0.8, 0.5, 0.5))


patch4_mat = patch_mat
patch4 = terrain.AddPatch(patch4_mat, 
                          chrono.ChCoordsysd(chrono.ChVector3d(50, 50, 0), chrono.QUNIT), 
                          100, 100)
patch4.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.8))


bump_mat = patch_mat
bump = terrain.AddPatch(bump_mat, 
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT), 
                        5, 5)
bump.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 10, 10)
bump.SetColor(chrono.ChColor(0.8, 0.5, 0.5))

terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
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


render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
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