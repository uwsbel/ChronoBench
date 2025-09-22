import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math

class ARTcar:
    def __init__(self, wheel_diameter, wheel_separation, wheel_radius, max_speed, max_acc, max_steering_angle):
        self.wheel_diameter = wheel_diameter
        self.wheel_separation = wheel_separation
        self.wheel_radius = wheel_radius
        self.max_speed = max_speed
        self.max_acc = max_acc
        self.max_steering_angle = max_steering_angle
        self.speed = 0
        self.steering_angle = 0

    def update(self, dt):
        self.speed += self.max_acc * dt
        self.speed = min(self.speed, self.max_speed)
        self.steering_angle += self.max_steering_angle * dt
        self.steering_angle = min(self.steering_angle, self.max_steering_angle)

def main():
    
    
    
    system = chrono.ChSystemNSC()

    
    car = ARTcar(0.25, 0.5, 0.05, 20.0, 1.0, math.pi / 6)
    car_pos = chrono.ChVectorD(0, 0, 0)
    car_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    car_body = chrono.ChBody()
    car_body.SetPos(car_pos)
    car_body.SetRot(car_rot)
    car_body.SetBodyFixed(False)
    car_body.SetCollide(True)
    car_body.SetMass(1000)
    car_body.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
    car_body.SetBodyType(chrono.ChBodyType.DYNAMIC)
    system.Add(car_body)

    
    driver = chrono.ChDriverSteeringWheelsDynamics()
    driver.Initialize(car_body, chrono.ChFrameD(chrono.ChVectorD(0.25, 0, 0), chrono.Q_from_AngAxis(math.pi / 6, chrono.ChVectorD(0, 1, 0))))
    driver.SetMaxSteeringAngle(math.pi / 6)
    driver.SetMaxSpeed(20.0)
    driver.SetMaxAcceleration(1.0)
    system.Add(driver)

    
    terrain = chrono.ChTerrain()
    terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
    terrain.SetTexture(chrono.GetChronoDataFile("textures/terrain.png"))
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.SetHeight(0.05)
    terrain.SetWidth(10.0)
    terrain.SetLength(10.0)
    terrain.SetOffset(0.0)
    terrain.SetDensity(1000)
    terrain.SetViscosity(0.0)
    terrain.SetFriction(0.8)
    terrain.SetRestitution(0.0)
    system.Add(terrain)

    
    lidar3d = sens.ChLidarSensor()
    lidar3d.Initialize(car_body, 5.0, chrono.ChFramed(chrono.ChVectorD(1.0, 0, 1), chrono.Q_from_Angle(math.pi / 6)), 800, 300, 2 * math.pi, -math.pi / 6, math.pi / 6, 100.0, sens.LidarBeamShape_RECTANGULAR, 1, 0.003, sens.LidarReturnMode_STRONGEST_RETURN)
    lidar3d.SetName("Lidar Sensor 3D")
    lidar3d.SetLag(0)
    lidar3d.SetCollectionWindow(1.0 / 5.0)
    lidar3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar3d.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth Data"))
    lidar3d.PushFilter(sens.ChFilterDIAccess())
    lidar3d.PushFilter(sens.ChFilterPCfromDepth())
    lidar3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar3d.PushFilter(sens.ChFilterXYZIAccess())
    system.AddSensor(lidar3d)

    lidar2d = sens.ChLidarSensor()
    lidar2d.Initialize(car_body, 5.0, chrono.ChFramed(chrono.ChVectorD(1.0, 0, 1), chrono.Q_from_Angle(math.pi / 6)), 800, 1, 2 * math.pi, 0, 0, 100.0, sens.LidarBeamShape_RECTANGULAR, 1, 0.003, sens.LidarReturnMode_STRONGEST_RETURN)
    lidar2d.SetName("Lidar Sensor 2D")
    lidar2d.SetLag(0)
    lidar2d.SetCollectionWindow(1.0 / 5.0)
    lidar2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw 2D Lidar Depth Data"))
    lidar2d.PushFilter(sens.ChFilterDIAccess())
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())
    system.AddSensor(lidar2d)

    
    camera = sens.ChCameraSensor()
    camera.Initialize(car_body, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0)
    camera.SetName("Camera Sensor")
    camera.SetLag(0)
    camera.SetCollectionWindow(1.0)
    system.AddSensor(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        dt = 1e-3
        car.update(dt)

        
        lidar3d.SetOffsetPose(chrono.ChFramed(chrono.ChVectorD(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1), chrono.Q_from_Angle(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))))
        lidar2d.SetOffsetPose(chrono.ChFramed(chrono.ChVectorD(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1), chrono.Q_from_Angle(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))))

        
        camera.SetOffsetPose(chrono.ChFramed(chrono.ChVectorD(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1.5), chrono.Q_from_Angle(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))))

        
        xyzi_buffer_3d = lidar3d.GetMostRecentXYZIBuffer()
        xyzi_buffer_2d = lidar2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_3d.HasData():
            xyzi_data_3d = xyzi_buffer_3d.GetXYZIData()
            print('XYZI buffer received from lidar 3D. Lidar resolution: {0}x{1}'.format(xyzi_buffer_3d.Width, xyzi_buffer_3d.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data_3d)))

        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print('XYZI buffer received from lidar 2D. Lidar resolution: {0}x{1}'.format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data_2d)))

        
        system.DoStepDynamics(step_size)

        
        ch_time = system.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)







noise_model = "NONE"  


return_mode = sens.LidarReturnMode_STRONGEST_RETURN





update_rate = 5.0


horizontal_samples = 800
vertical_samples = 300


horizontal_fov = 2 * math.pi  
max_vert_angle = math.pi / 12
min_vert_angle = -math.pi / 6


lag = 0


collection_time = 1. / update_rate  


sample_radius = 2


divergence_angle = 0.003






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"






main()