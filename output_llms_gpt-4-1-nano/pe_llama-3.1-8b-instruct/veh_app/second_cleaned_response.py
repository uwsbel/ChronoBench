import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

print(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, -5, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 1000


render_step_size = 1.0 / 50  


noise_model = "NONE"  


update_rate = 10


image_width = 1280
image_height = 720


fov = 1.408


lag = 0


exposure_time = 0


vis = True


gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
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


print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")


gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


box = chrono.ChBody()
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box_shape = chrono.ChVisualShapeBox(1, 1, 1)
box_shape.SetColor(chrono.ChColor(0, 0, 1))
box.AddVisualShape(box_shape)
gator.GetSystem().Add(box)


cylinder = chrono.ChBody()
cylinder.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder_shape = chrono.ChVisualShapeCylinder(0.5, 1)
cylinder_shape.SetColor(chrono.ChColor(0, 0, 1))
cylinder.AddVisualShape(cylinder_shape)
gator.GetSystem().Add(cylinder)


lidar = sens.ChLidarSensor(gator.GetSystem())
lidar.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(0.0, 0, 2)))
lidar.SetHorizontalSamples(800)
lidar.SetVerticalChannels(300)
lidar.SetHorizontalFOV(2 * chrono.CH_PI)
lidar.SetVerticalFOV(chrono.CH_PI / 12)
lidar.SetMaxRange(100.0)
lidar.SetBeamShape(chrono.ChVector3d(2, 2, 2))
lidar.SetSampleRadius(2)
lidar.SetDivergenceAngle(0.003)
lidar.SetStrongestReturnMode(True)
lidar.AddFilter(sens.ChFilterXYZI())
lidar.AddFilter(sens.ChFilterDepth())
lidar.AddFilter(sens.ChFilterIntensity())
lidar.AddFilter(sens.ChFilterVisualize(image_width, image_height, "Lidar Sensor"))
gator.GetSystem().Add(lidar)


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


realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    gator.Synchronize(time, driver_inputs)

    manager.Update()

    
    driver.Advance(step_size)
    gator.Advance(step_size)

    
    realtime_timer.Spin(step_size)