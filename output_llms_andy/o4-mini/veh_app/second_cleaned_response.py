import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math




def BlueColorAsset():
    c = chrono.ChColorAsset()
    c.SetColor(chrono.ChColor(0, 0, 1))
    return c




veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(0, -5, 0.4)                       
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


chassis_vis_type    = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type   = veh.VisualizationType_PRIMITIVES
wheel_vis_type      = veh.VisualizationType_NONE
tire_vis_type       = veh.VisualizationType_MESH


step_size      = 1e-3
tire_step_size = step_size
render_step    = 1.0/50
end_time       = 30


steer_input   = 0.5
throttle_input= 0.2


gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)

print("Vehicle mass:   ", gator.GetVehicle().GetMass())
print("Driveline type: ", gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     ", gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      ", gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("")

gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()





box = chrono.ChBodyEasyBox(1.0, 1.0, 1.0,     
                           1000,             
                           True, True)       
box.SetPos(chrono.ChVectorD(0, 0, 0.5))      
box.SetBodyFixed(True)
box.AddAsset(BlueColorAsset())
gator.GetSystem().Add(box)

cyl = chrono.ChBodyEasyCylinder(0.5, 1.0,    
                                1000,        
                                True, True)
cyl.SetPos(chrono.ChVectorD(0, 0, 1.5))      
cyl.SetBodyFixed(True)
cyl.AddAsset(BlueColorAsset())
gator.GetSystem().Add(cyl)




driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()




manager = sens.ChSensorManager(gator.GetSystem())


manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(1,1,1),
                            500.0)


cam_offset = chrono.ChFrameD(chrono.ChVectorD(-8.0, 0.0, 1.45),
                             chrono.ChQuaternionD(1,0,0,0))
camera = sens.ChCameraSensor(
    parent=gator.GetChassisBody(),
    updateRate=10.0,
    offsetPose=cam_offset,
    width=1280,
    height=720,
    fov=1.408
)
camera.SetName("ThirdPersonCam")
camera.PushFilter(sens.ChFilterVisualize(1280,720,"Gator Camera"))
manager.AddSensor(camera)


lidar_offset = chrono.ChFrameD(chrono.ChVector3D(0.0, 0.0, 2.0),
                               chrono.ChQuaternionD(1,0,0,0))
lidar = sens.ChLidarSensor(
    parent=gator.GetChassisBody(),
    updateRate=10.0,
    offsetPose=lidar_offset,
    horizontalSamples=800,
    verticalSamples=300,
    horizontalFOV=2 * chrono.CH_C_PI,
    verticalFOVUp=   chrono.CH_C_PI/12,
    verticalFOVDown=-chrono.CH_C_PI/6,
    maxRange=100.0
)

lidar.SetBeamShape(sens.ChLidarSensor.BeamShape_RECTANGULAR)
lidar.SetLaserScanRadius(2.0)
lidar.SetBeamDivergenceAngle(0.003)
lidar.SetReturnMode(sens.ChLidarSensor.ReturnMode_STRONGEST)

lidar.PushFilter(sens.ChFilterLidarDepth())
lidar.PushFilter(sens.ChFilterLidarIntensity())
lidar.PushFilter(sens.ChFilterLidarXYZI())
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Lidar Scan"))
manager.AddSensor(lidar)




realtime_timer = chrono.ChRealtimeStepTimer()
time = 0.0

while time < end_time:
    time = gator.GetSystem().GetChTime()

    
    driver.SetSteering(steer_input)
    driver.SetThrottle(throttle_input)

    
    inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, inputs, terrain)
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    
    realtime_timer.Spin(step_size)