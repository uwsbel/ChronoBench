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


driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()


manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)


box_body = chrono.ChBody()
box_body.SetPos(chrono.ChVector3d(0, 0, 0.5))
box_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(1, 1, 1))
box_shape.SetColor(chrono.ChColor(0, 0, 1))  
box_body.AddVisualShape(box_shape)
gator.GetSystem().Add(box_body)


cylinder_body = chrono.ChBody()
cylinder_body.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder_shape = chrono.ChVisualShapeCylinder(0.5, 1)
cylinder_shape.SetColor(chrono.ChColor(0, 0, 1))  
cylinder_body.AddVisualShape(cylinder_shape)
gator.GetSystem().Add(cylinder_body)


offset_pose = chrono.ChFramed(chrono.ChVector3d(0.0, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    10,  
    offset_pose,
    800,  
    300,  
    2 * chrono.CH_PI,  
    chrono.CH_PI / 12,  
    -chrono.CH_PI / 6,  
    100.0  
)
lidar.SetBeamShape(sens.ChBeamShape_Rectangular)
lidar.SetSampleRadius(2)
lidar.SetDivergenceAngle(0.003)
lidar.SetReturnMode(sens.ChLidarSensor.ReturnMode_Strongest)
lidar.PushFilter(sens.ChFilter_Depth())
lidar.PushFilter(sens.ChFilter_Intensity())
lidar.PushFilter(sens.ChFilter_XYZI_PointCloud())
lidar.PushFilter(sens.ChFilter_Visualize(chrono.ChColor(1, 1, 0)))
manager.AddSensor(lidar)


realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver.GetInputs(), terrain)

    manager.Update()

    driver.Advance(1e-3)
    terrain.Advance(1e-3)
    gator.Advance(1e-3)

    realtime_timer.Spin(1e-3)