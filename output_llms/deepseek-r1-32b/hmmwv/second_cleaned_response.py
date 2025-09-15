import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


terrainLength = 200.0  

class PathFollower:
    def __init__(self, radius=20.0, speed=5.0):
        self.radius = radius
        self.speed = speed
        self.angle = 0.0
        self.sentinel = None
        self.target = None

    def get_desired_state(self, time):
        angle = self.angle + (self.speed / self.radius) * time
        x = self.radius * math.cos(angle)
        y = self.radius * math.sin(angle)
        heading = angle + math.pi/2  
        return (x, y, heading)

    def update_spheres(self, system, time):
        if self.sentinel is None:
            self.sentinel = chrono.ChSphereShape()
            self.sentinel.SetRadius(0.5)
            self.sentinel.SetPos(chrono.ChVector3d(self.radius, 0, 1))
            self.sentinel.SetColor(chrono.ChColor(1, 0, 0))
            system.GetVisualSystem().AddShape(self.sentinel)
            
            self.target = chrono.ChSphereShape()
            self.target.SetRadius(0.5)
            self.target.SetPos(chrono.ChVector3d(self.radius, 0, 1))
            self.target.SetColor(chrono.ChColor(0, 1, 0))
            system.GetVisualSystem().AddShape(self.target)
        
        x, y, _ = self.get_desired_state(time)
        self.sentinel.SetPos(chrono.ChVector3d(x, y, 1))
        self.target.SetPos(chrono.ChVector3d(x + self.radius/2, y, 1))


path_follower = PathFollower()


kp = 1.0
ki = 0.1
kd = 0.5
integral = 0.0
prev_error = 0.0


vehicle = veh.HMMWV_Full()



patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()







system = vehicle.GetSystem()
path_follower.update_spheres(system, 0)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    
    desired_x, desired_y, desired_heading = path_follower.get_desired_state(time)
    
    
    current_pos = vehicle.GetChassis().GetPos()
    current_heading = vehicle.GetChassis().GetRot().Q_toAngleZ()
    
    
    error = math.atan2(desired_y - current_pos.y, desired_x - current_pos.x) - current_heading
    error = math.atan2(math.sin(error), math.cos(error))  
    
    
    integral += error * step_size
    derivative = (error - prev_error) / step_size
    steering = kp * error + ki * integral + kd * derivative
    steering = max(min(steering, 1.0), -1.0)  
    
    
    throttle = 0.3
    
    
    path_follower.update_spheres(system, time)
    
    
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = throttle
    driver_inputs.m_steering = steering
    
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    
    prev_error = error
    
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    step_number += 1
    realtime_timer.Spin(step_size)