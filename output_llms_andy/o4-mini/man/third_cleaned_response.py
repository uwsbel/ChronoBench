import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math





chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")




initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH
chassis_coll = veh.CollisionType_NONE
tire_model  = veh.TireModelType_TMEASY

terrainLength = 100.0
terrainWidth  = 100.0

contact_method = chrono.ChContactMethod_NSC
step_size       = 1e-3
render_fps      = 50
render_step     = 1.0 / render_fps

trackPoint = chrono.ChVector3d(-3, 0, 1.1)




vehicle = veh.MAN_10t()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_coll)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS:", vehicle.GetMass())





patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0),
                                            chrono.ChQuaterniond(1, 0, 0, 0)),
                         terrainLength, terrainWidth)


patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.4, 0.7, 0.4))
terrain.Initialize()





num_boxes = 10
for i in range(num_boxes):
    
    sx = np.random.uniform(0.3, 1.0)
    sy = np.random.uniform(0.3, 1.0)
    sz = np.random.uniform(0.3, 1.0)
    
    box = chrono.ChBodyEasyBox(sx, sy, sz,
                               2000,     
                               True,     
                               True)     
    
    x = np.random.uniform(-terrainLength/2 + sx, terrainLength/2 - sx)
    y = np.random.uniform(-terrainWidth/2  + sy, terrainWidth/2  - sy)
    box.SetPos(chrono.ChVectorD(x, y, 2.0 + sz/2))
    box.SetBodyFixed(False)
    vehicle.GetSystem().Add(box)




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN_10t + LiDAR + Boxes")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)




driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step / 1.0)
driver.SetThrottleDelta(render_step / 1.0)
driver.SetBrakingDelta(render_step / 0.3)
driver.Initialize()




sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
sensor_manager.SetVerbose(False)
sensor_manager.SetRenderRate(20)    


lidar = sens.ChLidarSensor(
    parent=vehicle.GetChassis(),       
    updateRate=10.0,                   
    pointRate=100000,                  
    horizontalSamples=512,
    verticalSamples=32,
    horizontalFOV=chrono.CH_C_PI,      
    verticalFOV=chrono.CH_C_PI / 6,    
    maxDistance=100.0,
    lag=chrono.ChFrame<double>(),      
)

lidar_pose = chrono.ChFrameD(
    chrono.ChVectorD(0.0, 0.0, 1.8), 
    chrono.ChQuaterniond(1, 0, 0, 0)
)
lidar.SetFrame(lidar_pose)
sensor_manager.AddSensor(lidar)




realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
render_steps   = math.ceil(render_step / step_size)

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    inputs = driver.GetInputs()

    
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vehicle.Synchronize(t, inputs, terrain)
    vis.Synchronize(t, inputs)

    
    sensor_manager.Update()      
    sensor_manager.Render()      
    sensor_manager.PostProcess()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)