import pychrono.core as chrono
import pychrono.vehicle as veh
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')





contact_method = chrono.ChContactMethod_NSC


vis_type             = veh.VisualizationType_MESH
chassis_collision    = veh.CollisionType_NONE
tire_model           = veh.TireModelType_TMEASY


terrainLength = 100.0
terrainWidth  = 100.0


step_size        = 1e-3
render_fps       = 50
render_step_size = 1.0 / render_fps
render_steps     = math.ceil(render_step_size / step_size)





initLoc = chrono.ChVector3d(-15, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)   

vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetChassisFixed(False)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", vehicle.GetMass())




patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0,0,0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Double Lane Change Demo')
vis.SetWindowSize(1280, 1024)


trackPoint = chrono.ChVector3d(3, 0, 2.1)
vis.SetChaseCamera(trackPoint, 25.0, 10.5)

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)




driver = veh.ChInteractiveDriverIRR(vis)

driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()




realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver.Synchronize(t)
    terrain.Synchronize(t)

    
    
    
    
    
    
    
    
    
    
    drv_inputs = driver.GetInputs()  
    
    drv_inputs.m_throttle = 0.6
    drv_inputs.m_braking  = 0.0
    
    if   t < 2.0:
        drv_inputs.m_steering = 0.0
    elif t < 2.8:
        drv_inputs.m_steering = +0.4
    elif t < 4.4:
        drv_inputs.m_steering = -0.4
    elif t < 5.2:
        drv_inputs.m_steering = +0.4
    elif t < 5.6:
        drv_inputs.m_steering = 0.0
    else:
        drv_inputs.m_steering = 0.0

    
    vehicle.Synchronize(t, drv_inputs, terrain)
    vis.Synchronize(t, drv_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)
    step_number += 1