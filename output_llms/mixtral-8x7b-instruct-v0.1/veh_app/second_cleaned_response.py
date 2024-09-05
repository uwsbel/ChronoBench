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


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
box.SetPos(chrono.ChVectorD(0, 0, 0.5))
box.SetCollide(True)
box.SetMaterialSurface(veh.GetDataFile("surfaces/blue.surf"))
gator.GetSystem().AddBody(box)


cylinder = chrono.ChBodyEasyCylinder(0.5, 1, 1000, True, True)
cylinder.SetPos(chrono.ChVectorD(0, 0, 1.5))
cylinder.SetCollide(True)
cylinder.SetMaterialSurface(veh.GetDataFile("surfaces/blue.surf"))
gator.GetSystem().AddBody(cylinder)


lidar_pose = chrono.ChFrameD(chrono.ChVectorD(0.0, 0, 2))
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    lidar_pose,
    image_width,
    image_height,
    fov,
    2 * math.pi,
    math.pi / 12,
    -math.pi / 6,
    100.0,
    sens.ChLidarSensor.BeamShape_RECTANGULAR,
    2,
    0.003,
    sens.ChLidarSensor.ReturnMode_STRONGEST,
    [sens.ChFilterDepth(), sens.ChFilterIntensity(), sens.ChFilterXYZI(), sens.ChFilterVisualize()]
)
lidar.SetName("Lidar Sensor")
manager.AddSensor(lidar)


realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    manager.Update()
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    realtime_timer.Spin(step_size)