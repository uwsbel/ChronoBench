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




terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()


box = chrono.ChBoxShape()
box.SetDimensions(chrono.ChVector3d(1, 1, 1))
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.SetColor(chrono.ChColor(0, 0, 1))
box.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.AddObject(box)


cylinder = chrono.ChCylinderShape()
cylinder.SetRadius(0.5)
cylinder.SetHeight(1)
cylinder.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder.SetColor(chrono.ChColor(0, 0, 1))
cylinder.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.AddObject(cylinder)


driver = veh.ChDriver(gator.GetVehicle())

driver.Initialize()




manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)


lidar_offset_pose = chrono.ChFrameD(chrono.ChVector3d(0.0, 0, 2), chrono.ChQuaterniond())
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
    chrono.ChRectangularBeamShape(),
    2,
    0.003,
    sens.ChLidarSensor.StrongestReturnMode,
    [sens.ChFilterDepth(), sens.ChFilterIntensity(), sens.ChFilterXYZI(), sens.ChFilterVisualize()]
)
lidar.SetName("Lidar Sensor")
manager.AddSensor(lidar)





realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    
print("error happened with only start ```python")