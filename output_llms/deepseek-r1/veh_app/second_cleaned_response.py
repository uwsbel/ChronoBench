import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, -5, 0.4)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH


gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(1e-3)
gator.Initialize()

gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)


terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, False)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().Add(box)


cylinder = chrono.ChBodyEasyCylinder(0.5, 1, 1000, True, False)
cylinder.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder.SetFixed(True)
cylinder.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().Add(cylinder)


manager = sens.ChSensorManager(gator.GetSystem())
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)


lidar_offset = chrono.ChFrameD(chrono.ChVector3d(0.0, 0, 2))
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    10,  
    lidar_offset,
    800,  
    300,  
    chrono.CH_2PI,  
    chrono.CH_PI/12,  
    -chrono.CH_PI/6,  
    100.0,  
    sens.LidarModel_RECTANGLE,
    2,  
    0.003,  
    sens.ReturnMode_STRONGEST_RETURN
)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Lidar View"))
manager.AddSensor(lidar)


driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()


realtime_timer = chrono.ChRealtimeStepTimer()
end_time = 30
while gator.GetSystem().GetChTime() < end_time:
    time = gator.GetSystem().GetChTime()
    
    
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver.GetInputs(), terrain)
    manager.Update()
    
    
    driver.Advance(0.001)
    terrain.Advance(0.001)
    gator.Advance(0.001)
    realtime_timer.Spin(0.001)