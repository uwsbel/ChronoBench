import pychrono as chrono
from pychrono import irrlicht
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, False)
ground.SetPos(chrono.ChVector(0, -0.5, 0))
ground.GetVisualShape().GetMesh().SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.SetBodyFixed(True)
system.Add(ground)


chassis = chrono.ChBodyEasyBox(2, 1, 0.5, 1000, True, False)
chassis.SetPos(chrono.ChVector(0, 0.25, 0))
chassis.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(chassis)


wheel_radius = 0.2
wheel_length = 0.4

front_left_wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_length, 1000, True, False)
front_left_wheel.SetPos(chrono.ChVector(1, -0.5, 0.25))
front_left_wheel.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(front_left_wheel)

front_right_wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_length, 1000, True, False)
front_right_wheel.SetPos(chrono.ChVector(1, 0.5, 0.25))
front_right_wheel.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(front_right_wheel)

rear_left_wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_length, 1000, True, False)
rear_left_wheel.SetPos(chrono.ChVector(-1, -0.5, 0.25))
rear_left_wheel.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(rear_left_wheel)

rear_right_wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_length, 1000, True, False)
rear_right_wheel.SetPos(chrono.ChVector(-1, 0.5, 0.25))
rear_right_wheel.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(rear_right_wheel)


joint_front_left = chrono.ChLinkRevolute()
joint_front_left.Initialize(chassis, front_left_wheel, chrono.ChVector(1, -0.5, 0.25), chrono.ChVector(0, 1, 0))
system.AddLink(joint_front_left)

joint_front_right = chrono.ChLinkRevolute()
joint_front_right.Initialize(chassis, front_right_wheel, chrono.ChVector(1, 0.5, 0.25), chrono.ChVector(0, 1, 0))
system.AddLink(joint_front_right)

joint_rear_left = chrono.ChLinkRevolute()
joint_rear_left.Initialize(chassis, rear_left_wheel, chrono.ChVector(-1, -0.5, 0.25), chrono.ChVector(1, 0, 0))
system.AddLink(joint_rear_left)

joint_rear_right = chrono.ChLinkRevolute()
joint_rear_right.Initialize(chassis, rear_right_wheel, chrono.ChVector(-1, 0.5, 0.25), chrono.ChVector(1, 0, 0))
system.AddLink(joint_rear_right)


application = irrlicht.ChIrrApp(system, "Viper Rover Simulation", irrlicht.dimension2d_i(800, 600))
application.AddCamera(chrono.ChVector(0, 2, 5))
application.AddLight(chrono.ChVector(0, 5, 5), 10)


while application.GetDevice().run():
    time = system.GetChTime()
    
    
    desired_angle_deg = 30 * (time / 10)
    desired_angle_rad = np.radians(desired_angle_deg)
    
    
    torque = 1000
    front_left_wheel.ApplyTorque(chrono.ChVector(0, torque, 0), True)
    front_right_wheel.ApplyTorque(chrono.ChVector(0, torque, 0), True)
    
    
    drive_torque = 5000
    rear_left_wheel.ApplyTorque(chrono.ChVector(0, 0, drive_torque), True)
    rear_right_wheel.ApplyTorque(chrono.ChVector(0, 0, drive_torque), True)
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(1/60)