import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np
import time






step_size = 1e-3


sensor_update_interval = 0.01  


t_end = 10.0






system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


vehicle = veh.HMMWV()
vehicle.Initialize(system, veh.VehicleSide.FRONT)


vehicle.GetChassis().SetPos(chrono.ChVectorD(0, 0, 0.5))
vehicle.GetChassis().SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


terrain = veh.RigidTerrain(system)
patch_size = 100
patch = veh.RigidTerrain.Patch(patch_size, patch_size, 0, patch_size/2, 0, patch_size/2)
patch.SetTexture(veh.RigidTerrain.Texture("terrain/textures/tile4.jpg"))
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.AddPatch(patch)
terrain.Initialize()


driver = veh.ChDataDriver()
driver.SetSteeringFunction(chrono.ChFunction_Const(0))  
driver.SetThrottleFunction(chrono.ChFunction_Const(0.3))  
driver.Initialize()






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Simulation with Sensors")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(8, 0, 2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddShadowAll()


vis.SetSymbolScale(0.01)
vis.SetContactDrawingMode(chronoirr.ChVisualSystemIrrlicht.ContactDrawingMode.CONTACT_FORCES)
vis.SetContactForceDrawingMode(chronoirr.ChVisualSystemIrrlicht.ContactForceDrawingMode.SPRINGS)





class IMUSensor:
    def __init__(self, chassis):
        self.chassis = chassis
        self.acceleration = chrono.ChVectorD(0, 0, 0)
        self.angular_velocity = chrono.ChVectorD(0, 0, 0)
        self.orientation = chrono.ChQuaternionD(1, 0, 0, 0)
        self.last_time = 0

    def update(self, time):
        if time - self.last_time >= sensor_update_interval:
            
            self.acceleration = self.chassis.GetPos_dt2()
            self.angular_velocity = self.chassis.GetWvel_loc()
            self.orientation = self.chassis.GetRot()

            
            noise = 0.01
            self.acceleration.x += np.random.uniform(-noise, noise)
            self.acceleration.y += np.random.uniform(-noise, noise)
            self.acceleration.z += np.random.uniform(-noise, noise)

            self.angular_velocity.x += np.random.uniform(-noise, noise)
            self.angular_velocity.y += np.random.uniform(-noise, noise)
            self.angular_velocity.z += np.random.uniform(-noise, noise)

            self.last_time = time

            
            print(f"IMU - Acceleration: {self.acceleration}")
            print(f"IMU - Angular Velocity: {self.angular_velocity}")
            print(f"IMU - Orientation: {self.orientation}")

class GPSSensor:
    def __init__(self, chassis):
        self.chassis = chassis
        self.position = chrono.ChVectorD(0, 0, 0)
        self.velocity = chrono.ChVectorD(0, 0, 0)
        self.last_time = 0

    def update(self, time):
        if time - self.last_time >= sensor_update_interval:
            
            self.position = self.chassis.GetPos()
            self.velocity = self.chassis.GetPos_dt()

            
            noise = 0.1
            self.position.x += np.random.uniform(-noise, noise)
            self.position.y += np.random.uniform(-noise, noise)
            self.position.z += np.random.uniform(-noise, noise)

            self.velocity.x += np.random.uniform(-noise, noise)
            self.velocity.y += np.random.uniform(-noise, noise)
            self.velocity.z += np.random.uniform(-noise, noise)

            self.last_time = time

            
            print(f"GPS - Position: {self.position}")
            print(f"GPS - Velocity: {self.velocity}")


imu = IMUSensor(vehicle.GetChassis())
gps = GPSSensor(vehicle.GetChassis())






time = 0
frame_count = 0
last_output_time = 0

while time < t_end:
    
    time = system.GetChTime()

    
    driver.SetTime(time)
    driver.Synchronize(time)

    
    vehicle.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())

    
    imu.update(time)
    gps.update(time)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if time - last_output_time >= 1.0:
        chassis_mass = vehicle.GetChassis().GetMass()
        print(f"Time: {time:.2f}s - Vehicle mass: {chassis_mass:.2f} kg")
        last_output_time = time

    
    system.DoStepDynamics(step_size)

    
    frame_count += 1





print("Simulation complete")
print(f"Total simulation time: {time:.2f} seconds")
print(f"Total frames rendered: {frame_count}")