import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Fix: Changed terrainLength to 200.0
terrainLength = 200.0  # Increased from 100.0

class PathFollower:
    def __init__(self, radius=20.0, speed=5.0):
        self.radius = radius
        self.speed = speed
        self.timestamp = 0.0
        
    def get_desired_state(self, time):
        angle = (self.speed / self.radius) * time
        x = self.radius * math.sin(angle)
        y = self.radius * math.cos(angle) - self.radius
        heading = angle
        return (x, y, heading)

# Create path follower
path_follower = PathFollower()

# Create visualization points
sentinel = irr.ChSphereShape(0.5, irr.ChColor(0,1,0))  # Green sphere
target = irr.ChSphereShape(0.5, irr.ChColor(1,0,0))    # Red sphere

# Add visualization points to scene
vis.GetSceneManager().addSceneNode(sentinel)
vis.GetSceneManager().addSceneNode(target)

# PID Controller parameters
kp = 1.0
ki = 0.1
kd = 0.5
integral = 0.0
prev_error = 0.0

# Modify driver inputs
driver_inputs = veh.ChDriverInputs()
driver_inputs.m_throttle = 0.3  # Constant throttle

# Main simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Get desired state from path follower
    desired_x, desired_y, desired_heading = path_follower.get_desired_state(time)
    
    # Get current vehicle position and heading
    current_pos = vehicle.GetVehicle().GetChassisBody().GetPos()
    current_heading = vehicle.GetVehicle().GetChassisBody().GetRot().Q_to_angle_z()
    
    # Calculate error for PID control
    error = math.atan2(desired_y - current_pos.y, desired_x - current_pos.x)
    error -= current_heading
    error = math.atan2(math.sin(error), math.cos(error))  # Keep error between -pi and pi
    
    # PID Control calculations
    integral += error * step_size
    derivative = (error - prev_error) / step_size
    steering = kp * error + ki * integral + kd * derivative
    steering = max(min(steering, 1.0), -1.0)  # Limit steering between -1 and 1
    
    # Update driver inputs
    driver_inputs.m_steering = steering
    
    # Update visualization points
    sentinel.SetPos(chrono.ChVector3d(desired_x, desired_y, 1.0))
    target.SetPos(chrono.ChVector3d(desired_x + math.cos(desired_heading)*10, 
                                   desired_y + math.sin(desired_heading)*10, 
                                   1.0))
    
    # Synchronize modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    # Update timestamps and errors
    prev_error = error
    path_follower.timestamp += step_size
    
    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    step_number += 1
    realtime_timer.Spin(step_size)