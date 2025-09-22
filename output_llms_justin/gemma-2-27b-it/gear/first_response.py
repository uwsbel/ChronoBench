import pychrono as chrono
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

# Create the physical system
system = chrono.ChSystemNSC()

# Set the gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddCameraLight()

# Create a fixed truss
truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
system.AddBody(truss)

# Create a rotating bar
bar = chrono.ChBody()
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
bar.SetPos(chrono.ChVectorD(0, 0, 0.5))
system.AddBody(bar)

# Create a revolute joint between the bar and the truss
joint_bar_truss = chrono.ChLinkRevolute()
joint_bar_truss.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_ROTATE_X_TO_Z))
system.AddLink(joint_bar_truss)

# Create the gears
gear1 = chrono.ChBodyEasyCylinder(0.2, 0.05, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(0.5, 0, 0))
system.AddBody(gear1)

gear2 = chrono.ChBodyEasyCylinder(0.1, 0.05, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(0.5, 0, 0.2))
system.AddBody(gear2)

# Create revolute joints for the gears
joint_gear1_bar = chrono.ChLinkRevolute()
joint_gear1_bar.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_X_TO_Z))
system.AddLink(joint_gear1_bar)

joint_gear2_gear1 = chrono.ChLinkRevolute()
joint_gear2_gear1.Initialize(gear1, gear2, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.2), chrono.Q_ROTATE_X_TO_Z))
system.AddLink(joint_gear2_gear1)

# Create a motor to rotate gear1
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_X_TO_Z))
motor.SetSpeedFunction(chrono.ChFunction_Const(10))
system.AddLink(motor)

# ---------------------------------------------------------------------
#
#  Simulation loop
#

while vis.Run():
    system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()