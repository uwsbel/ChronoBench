import pychrono as chrono
from pychrono import irrlicht
from pychrono.irrlicht import ChIrrApp
import numpy as np

# Create the Chrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

# Create the Irrlicht application
app = irrlicht.ChIrrApp(system, "Curiosity Rover Simulation", irrlicht.dimension2duint(800, 600))
app.SetCameraPosition(chrono.ChVectorD(0, 2, -5))
app.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
app.SetCameraVerticalDirection(chrono.ChVectorD(0, 1, 0))
app.SetFog(0.01, 0.9, 0.01, 0.9, 0.01, 0.9)
app.SetLightDirection(chrono.ChVectorD(0, 1, 0), 1.0, 0.5, 0.5, 0.5)
app.SetLightDirection(chrono.ChVectorD(0, -1, 0), 1.0, 0.5, 0.5, 0.5)
app.SetShadowDirection(chrono.ChVectorD(0, -1, 0), 0.5, 0.5, 0.5, 0.5)

# Create ground
ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetFixed(True)
ground.GetMaterialSurface(0).GetTexture().SetTexture(chrono.GetChronoDataFile("textures/ground.jpg"))
system.Add(ground)

# Create rover chassis
chassis = chrono.ChBodyEasyBox(2, 1, 1, 1000, True, True)
chassis.SetPos(chrono.ChVectorD(0, 0.5, 0))
chassis.SetFixed(False)
chassis.GetMaterialSurface(0).GetTexture().SetTexture(chrono.GetChronoDataFile("textures/rover_chassis.jpg"))
system.Add(chassis)

# Create wheels
wheel_radius = 0.3
wheel_length = 0.2
wheel_mass = 10

front_wheel1 = chrono.ChBodyEasyCylinder(wheel_radius, wheel_length, wheel_mass, True, True)
front_wheel1.SetPos(chrono.ChVectorD(1, 0.5, 0))
front_wheel1.SetFixed(False)
front_wheel1.GetMaterialSurface(0).GetTexture().SetTexture(chrono.GetChronoDataFile("textures/rover_wheel.jpg"))
system.Add(front_wheel1)

front_wheel2 = chrono.ChBodyEasyCylinder(wheel_radius, wheel_length, wheel_mass, True, True)
front_wheel2.SetPos(chrono.ChVectorD(-1, 0.5, 0))
front_wheel2.SetFixed(False)
front_wheel2.GetMaterialSurface(0).GetTexture().SetTexture(chrono.GetChronoDataFile("textures/rover_wheel.jpg"))
system.Add(front_wheel2)

rear_wheel1 = chrono.ChBodyEasyCylinder(wheel_radius, wheel_length, wheel_mass, True, True)
rear_wheel1.SetPos(chrono.ChVectorD(1, 0.5, 0.5))
rear_wheel1.SetFixed(False)
rear_wheel1.GetMaterialSurface(0).GetTexture().SetTexture(chrono.GetChronoDataFile("textures/rover_wheel.jpg"))
system.Add(rear_wheel1)

rear_wheel2 = chrono.ChBodyEasyCylinder(wheel_radius, wheel_length, wheel_mass, True, True)
rear_wheel2.SetPos(chrono.ChVectorD(-1, 0.5, 0.5))
rear_wheel2.SetFixed(False)
rear_wheel2.GetMaterialSurface(0).GetTexture().SetTexture(chrono.GetChronoDataFile("textures/rover_wheel.jpg"))
system.Add(rear_wheel2)

# Create joints between chassis and wheels
joint_front1 = chrono.ChLinkRevolute()
joint_front1.Initialize(chassis, front_wheel1, chrono.ChVectorD(1, 0.5, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.AddLink(joint_front1)

joint_front2 = chrono.ChLinkRevolute()
joint_front2.Initialize(chassis, front_wheel2, chrono.ChVectorD(-1, 0.5, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.AddLink(joint_front2)

joint_rear1 = chrono.ChLinkRevolute()
joint_rear1.Initialize(chassis, rear_wheel1, chrono.ChVectorD(1, 0.5, 0.5), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.AddLink(joint_rear1)

joint_rear2 = chrono.ChLinkRevolute()
joint_rear2.Initialize(chassis, rear_wheel2, chrono.ChVectorD(-1, 0.5, 0.5), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.AddLink(joint_rear2)

# Create motors for wheels
motor_front1 = chrono.ChLinkMotorTorque()
motor_front1.Initialize(joint_front1, chrono.ChVectorD(0, 0, 1), 0, 0)
system.AddLink(motor_front1)

motor_front2 = chrono.ChLinkMotorTorque()
motor_front2.Initialize(joint_front2, chrono.ChVectorD(0, 0, 1), 0, 0)
system.AddLink(motor_front2)

motor_rear1 = chrono.ChLinkMotorTorque()
motor_rear1.Initialize(joint_rear1, chrono.ChVectorD(0, 0, 1), 0, 0)
system.AddLink(motor_rear1)

motor_rear2 = chrono.ChLinkMotorTorque()
motor_rear2.Initialize(joint_rear2, chrono.ChVectorD(0, 0, 1), 0, 0)
system.AddLink(motor_rear2)

# Input handling
forward = False
backward = False
steer_left = False
steer_right = False

def OnKeyDown(event):
    global forward, backward, steer_left, steer_right
    key = event.GetKey()
    if key == irrlicht.KEY_W:
        forward = True
    elif key == irrlicht.KEY_S:
        backward = True
    elif key == irrlicht.KEY_A:
        steer_left = True
    elif key == irrlicht.KEY_D:
        steer_right = True

def OnKeyUp(event):
    global forward, backward, steer_left, steer_right
    key = event.GetKey()
    if key == irrlicht.KEY_W:
        forward = False
    elif key == irrlicht.KEY_S:
        backward = False
    elif key == irrlicht.KEY_A:
        steer_left = False
    elif key == irrlicht.KEY_D:
        steer_right = False

app.SetKeyDownCallback(OnKeyDown)
app.SetKeyUpCallback(OnKeyUp)

# Simulation loop
while app.GetApplication().IsRunning():
    app.BeginScene(True, True, irrlicht.SColor(255, 100, 100, 100))
    app.DrawAll()
    app.EndScene()
    app.DoStep()

    # Update motor torques based on input
    torque = 100.0
    if forward:
        motor_front1.SetTorque(torque)
        motor_front2.SetTorque(torque)
        motor_rear1.SetTorque(torque)
        motor_rear2.SetTorque(torque)
    elif backward:
        motor_front1.SetTorque(-torque)
        motor_front2.SetTorque(-torque)
        motor_rear1.SetTorque(-torque)
        motor_rear2.SetTorque(-torque)
    else:
        motor_front1.SetTorque(0)
        motor_front2.SetTorque(0)
        motor_rear1.SetTorque(0)
        motor_rear2.SetTorque(0)