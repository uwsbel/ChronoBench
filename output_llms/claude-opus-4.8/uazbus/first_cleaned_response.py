import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')






initLoc = chrono.ChVector3d(0, 0, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_NSC


step_size = 1e-3






uazbus = veh.UAZBUS()
uazbus.SetContactMethod(contact_method)
uazbus.SetChassisCollisionType(chassis_collision_type)
uazbus.SetChassisFixed(False)
uazbus.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
uazbus.SetTireType(tire_model)
uazbus.SetTireStepSize(step_size)


uazbus.Initialize()


uazbus.SetChassisVisualizationType(vis_type)
uazbus.SetSuspensionVisualizationType(vis_type)
uazbus.SetSteeringVisualizationType(vis_type)
uazbus.SetWheelVisualizationType(vis_type)
uazbus.SetTireVisualizationType(vis_type)





patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)        
patch_mat.SetRestitution(0.01)    

terrain = veh.RigidTerrain(uazbus.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength,
    terrainWidth
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Vehicle Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(uazbus.GetVehicle())





driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0   
throttle_time = 1.0   
braking_time  = 0.3   
driver.SetSteeringDelta(render_step_size := step_size / steering_time)
driver.SetThrottleDelta(step_size / throttle_time)
driver.SetBrakingDelta(step_size / braking_time)

driver.Initialize()






print("VEHICLE MASS: ", uazbus.GetVehicle().GetMass())


realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = uazbus.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    uazbus.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    uazbus.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)