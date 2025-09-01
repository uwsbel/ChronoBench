import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')




initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)




vis_type             = veh.VisualizationType_MESH
chassis_collision   = veh.CollisionType_NONE
tire_model          = veh.TireModelType_TMEASY




terrainLength = 100.0
terrainWidth  = 100.0




contact_method    = chrono.ChContactMethod_NSC
step_size         = 1e-3
tire_step_size    = step_size
render_fps        = 50
render_step_size  = 1.0 / render_fps




vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)


vehicle.SetMaxMotorVoltageRatio(0.26)     
vehicle.SetStallTorque(0.4)               
vehicle.SetTireRollingResistance(0.03)    

vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(
    chrono.ChCollisionSystemType_BULLET)




mat_NS = chrono.ChMaterialSurfaceNSC()
mat_NS.SetFriction(0.9)
mat_NS.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    mat_NS,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('ARTcar Simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0,0,0.2), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)                




driver = veh.ChInteractiveDriverIRR(vis)
steer_time   = 1.0   
throttle_time= 1.0   
brake_time   = 0.3   
driver.SetSteeringDelta(render_step_size / steer_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / brake_time)
driver.Initialize()




print("VEHICLE MASS:", vehicle.GetChassisBody().GetMass())

realtime_timer = chrono.ChRealtimeStepTimer()
render_steps   = math.ceil(render_step_size / step_size)
step_number    = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vehicle.Synchronize(t, driver_inputs, terrain)
    vis.Synchronize(t, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)
    step_number += 1