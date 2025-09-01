import math
import pychrono.core    as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh




chrono.SetChronoDataPath(chrono.GetChronoDataPath())          
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      




initLoc = chrono.ChVector3d(0.0, 0.0, 0.5)
initRot = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.ChassisCollisionType_PRIMITIVES


tire_model            = veh.TireModelType_TMEASY


terrainHeight = 0.0
terrainLength = 100.0
terrainWidth  = 100.0


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC


step_size       = 1e-3
tire_step_size  = step_size
render_fps      = 50
render_step_size = 1.0 / render_fps




gator = veh.Gator()
gator.SetContactMethod(contact_method)
gator.SetChassisCollisionType(chassis_collision_type)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(tire_model)
gator.SetTireStepSize(tire_step_size)
gator.Initialize()


gator.SetChassisVisualizationType(vis_type)
gator.SetSuspensionVisualizationType(vis_type)
gator.SetSteeringVisualizationType(vis_type)
gator.SetWheelVisualizationType(vis_type)
gator.SetTireVisualizationType(vis_type)


gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




material = chrono.ChContactMaterialNSC()
material.SetFriction(0.9)
material.SetRestitution(0.01)

terrain = veh.RigidTerrain(gator.GetSystem())
patch = terrain.AddPatch(material,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator – primitive visualisation & collision")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 2.5   
throttle_time = 2.0   
braking_time  = 1.0   

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()




print("VEHICLE MASS:", gator.GetVehicle().GetMass())

render_steps   = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
render_frame   = 0

while vis.Run():
    time = gator.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)