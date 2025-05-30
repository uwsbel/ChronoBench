import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math





chrono.SetChronoDataPath(chrono.GetChronoDataPath())

veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")





initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH

chassis_collision_type = veh.CollisionType_NONE

tire_model = veh.TireModelType_RIGID


terrainHeight = 0.0
terrainLength = 100.0
terrainWidth  = 100.0


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC


step_size       = 1e-3
tire_step_size  = step_size
render_fps      = 50
render_step_size = 1.0 / render_fps




vehicle = veh.UAZBUS()
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


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())





patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength,
    terrainWidth
)

patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()





obstacle = chrono.ChBodyEasyBox(
    0.5,  
    5.0,  
    0.2,  
    1000, 
    True, 
    True  
)
obstacle.SetBodyFixed(True)
obstacle.SetPos(chrono.ChVector3d(5, 0, 0.1))
vehicle.GetSystem().Add(obstacle)





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()

vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)

driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()




realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = math.ceil(render_step_size / step_size)
step_number = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver.Synchronize(t)
    inputs = driver.GetInputs()
    
    inputs.m_throttle = 0.5
    
    
    

    
    terrain.Synchronize(t)
    vehicle.Synchronize(t, inputs, terrain)
    vis.Synchronize(t, inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)
    step_number += 1