import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Global parameters
rpm = 60  # Motor rotation speed in RPM
gear_thickness = 0.1
gear_1_teeth = 32  # Sun gear
gear_2_teeth = 16  # Planet gear
mod = 0.001  # Module for the gear teeth

# Create the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a fixed truss (ground)
truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(truss)

# Create materials
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)

# Create the sun gear - central gear
gear_1_radius = mod * gear_1_teeth / 2
gear_1 = chrono.ChBody()
gear_1.SetPos(chrono.ChVectorD(0, 0, 0))
gear_1.SetMass(1.0)
gear_1.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))

# Add visualization geometry for the sun gear
gear_1_shape = chrono.ChCylinderShape()
gear_1_shape.GetCylinderGeometry().rad = gear_1_radius
gear_1_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -gear_thickness/2)
gear_1_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, gear_thickness/2)
gear_1.AddVisualShape(gear_1_shape, chrono.ChFrameD())

gear_1_teeth_vis = chrono.ChTorusShape()
gear_1_teeth_vis.GetTorusGeometry().rad = 0.02
gear_1_teeth_vis.GetTorusGeometry().radt = gear_1_radius
gear_1.AddVisualShape(gear_1_teeth_vis)

# Add a small marker at the center of the sun gear
marker = chrono.ChSphereShape()
marker.GetSphereGeometry().rad = 0.02
gear_1.AddVisualShape(marker)

system.Add(gear_1)

# Create a revolute joint between the truss and the sun gear
joint_1 = chrono.ChLinkLockRevolute()
joint_1.Initialize(truss, gear_1, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                                    chrono.Q_from_AngZ(0)))
system.Add(joint_1)

# Create the rotating arm (carrier)
arm_length = gear_1_radius + gear_1_radius/2 + gear_1_radius  # Enough space for gears
arm = chrono.ChBody()
arm.SetPos(chrono.ChVectorD(0, 0, 0))
arm.SetMass(1.5)
arm.SetInertiaXX(chrono.ChVectorD(0.2, 0.2, 0.2))

# Add visualization geometry for the arm
arm_shape = chrono.ChBoxShape()
arm_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(arm_length * 2, 0.05, 0.05))
arm.AddVisualShape(arm_shape)

system.Add(arm)

# Create a revolute joint between the truss and the arm
joint_arm = chrono.ChLinkLockRevolute()
joint_arm.Initialize(truss, arm, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                                   chrono.Q_from_AngZ(0)))
system.Add(joint_arm)

# Create the planet gear
gear_2_radius = mod * gear_2_teeth / 2
planet_pos = chrono.ChVectorD(arm_length, 0, 0)
gear_2 = chrono.ChBody()
gear_2.SetPos(planet_pos)
gear_2.SetMass(0.5)
gear_2.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))

# Add visualization geometry for the planet gear
gear_2_shape = chrono.ChCylinderShape()
gear_2_shape.GetCylinderGeometry().rad = gear_2_radius
gear_2_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -gear_thickness/2)
gear_2_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, gear_thickness/2)
gear_2.AddVisualShape(gear_2_shape, chrono.ChFrameD())

gear_2_teeth_vis = chrono.ChTorusShape()
gear_2_teeth_vis.GetTorusGeometry().rad = 0.01
gear_2_teeth_vis.GetTorusGeometry().radt = gear_2_radius
gear_2.AddVisualShape(gear_2_teeth_vis)

system.Add(gear_2)

# Create a revolute joint between the arm and the planet gear
joint_2 = chrono.ChLinkLockRevolute()
joint_2.Initialize(arm, gear_2, chrono.ChCoordsysD(planet_pos, chrono.Q_from_AngZ(0)))
system.Add(joint_2)

# Create a gear constraint between gear_1 and gear_2
gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(gear_1, gear_2, 
                          chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)), 
                          chrono.ChFrameD(planet_pos))
gear_constraint.SetTransmissionRatio(-gear_1_teeth / gear_2_teeth)  # Negative for internal mesh
system.Add(gear_constraint)

# Create a motor for the arm
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, arm, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngZ(0)))
motor_speed = chrono.ChFunction_Const(rpm * np.pi / 30)  # Convert RPM to rad/s
motor.SetSpeedFunction(motor_speed)
system.Add(motor)

# Create the visualization window
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Epicyclic Gear System")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Set colors for visualization
gear_1_col = chrono.ChColor(0.6, 0.2, 0.2)
gear_2_col = chrono.ChColor(0.2, 0.6, 0.2)
arm_col = chrono.ChColor(0.2, 0.2, 0.6)

gear_1.GetVisualShape(0).SetColor(gear_1_col)
gear_2.GetVisualShape(0).SetColor(gear_2_col)
arm.GetVisualShape(0).SetColor(arm_col)

# Simulation parameters
time_step = 0.002
total_time = 10.0  # seconds to simulate

# Simulation loop
while vis.Run() and system.GetChTime() < total_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)

print("Simulation completed")