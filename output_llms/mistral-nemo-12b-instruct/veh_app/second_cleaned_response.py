import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, -5, 0.4)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)




gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()




gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()


box_mat = chrono.ChMaterialSurfaceNSC()
box_mat.SetFriction(0.6)
box_mat.SetRestitution(0.3)
box = chrono.ChBoxShape(1, 1, 1)
box_mat.SetContactMethod(chrono.ChContactMethod_NSC)
box_mat.SetSmoothness(0.8)
box_mat.SetTexture(veh.GetDataFile("textures/blue.jpg"))
box_body = chrono.ChBody()
box_body.SetMaterial(box_mat)
box_body.SetPos(chrono.ChVector3d(0, 0, 0.5))
box_body.SetBodyFixed(True)
box_body.AddShape(box, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0)))
gator.GetSystem().Add(box_body)


cylinder_mat = chrono.ChMaterialSurfaceNSC()
cylinder_mat.SetFriction(0.6)
cylinder_mat.SetRestitution(0.3)
cylinder = chrono.ChCylinderShape(0.5, 1)
cylinder_mat.SetContactMethod(chrono.ChContactMethod_NSC)
cylinder_mat.SetSmoothness(0.8)
cylinder_mat.SetTexture(veh.GetDataFile("textures/blue.jpg"))
cylinder_body = chrono.ChBody()
cylinder_body.SetMaterial(cylinder_mat)
cylinder_body.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder_body.SetBodyFixed(True)
cylinder_body.AddShape(cylinder, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0)))
gator.GetSystem().Add(cylinder_body)




driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()




manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)


offset_pose = chrono.ChFramed(chrono.ChVector3d(-8.0, 0, 1.45), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov
)
cam.SetName("Third Person POV")

cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)


lidar_offset_pose = chrono.ChFramed(chrono.ChVector3d(0.0, 0, 2))
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    lidar_offset_pose,
    800,
    300,
    2 * chrono.CH_PI,
    chrono.CH_PI / 12,
    -chrono.CH_PI / 6,
    100.0,
    sens.ChLidarSensor.BeamShape_RECTANGULAR,
    2,
    0.003,
    sens.ChLidarSensor.ReturnMode_STRONGEST
)
lidar.PushFilter(sens.ChFilterDepth())
lidar.PushFilter(sens.ChFilterIntensity())
lidar.PushFilter(sens.ChFilterXYZI())
lidar.PushFilter(sens.ChFilterVisualizeXYZI())
manager.AddSensor(lidar)






realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    
    driver.SetSteering(0.5)  
    driver.SetThrottle(0.2)