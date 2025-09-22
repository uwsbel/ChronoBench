import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the ChSystem
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a fixed truss
truss = chrono.ChBody()
truss.SetBodyFixed(True)
system.Add(truss)

# Create the rotating bar
bar = chrono.ChBody()
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetMass(1.0)
bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(bar)

# Create a revolute joint to connect the bar to the truss
rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.AddLink(rev_joint)

# Create two gears
gear1 = chrono.ChBody()
gear1.SetPos(chrono.ChVectorD(0.2, 0, 0))
gear1.SetMass(0.5)
gear1.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))
system.Add(gear1)

gear2 = chrono.ChBody()
gear2.SetPos(chrono.ChVectorD(0.4, 0, 0))
gear2.SetMass(0.5)
gear2.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))
system.Add(gear2)

# Create revolute joints for the gears
rev_joint1 = chrono.ChLinkLockRevolute()
rev_joint1.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVectorD(0.2, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.AddLink(rev_joint1)

rev_joint2 = chrono.ChLinkLockRevolute()
rev_joint2.Initialize(bar, gear2, chrono.ChCoordsysD(chrono.ChVectorD(0.4, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.AddLink(rev_joint2)

# Create gear constraint between the two gears
gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(gear1, gear2, chrono.ChFrameD(chrono.ChVectorD(0.2, 0, 0)), chrono.ChFrameD(chrono.ChVectorD(0.4, 0, 0)))
gear_constraint.SetTransmissionRatio(-1)  # Negative ratio for opposite rotation
system.AddLink(gear_constraint)

# Create a motor for gear1
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, gear1, chrono.ChFrameD(chrono.ChVectorD(0.2, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  # Constant speed of 1 rad/s
system.AddLink(motor)

# Create the Irrlicht visualization
application = chronoirr.ChIrrApp(system, "Epicyclic Gear System", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1, 1, 1))

# Add visual assets
visual_material = chrono.ChVisualMaterial()
visual_material.SetDiffuseColor(chrono.ChColor(0.8, 0.3, 0.3))

truss_shape = chrono.ChBoxShape()
truss_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.1)
truss.AddVisualShape(truss_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

bar_shape = chrono.ChCylinderShape()
bar_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -0.2)
bar_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0.5, 0, -0.2)
bar_shape.GetCylinderGeometry().rad = 0.02
bar.AddVisualShape(bar_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

gear_shape = chrono.ChCylinderShape()
gear_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -0.05)
gear_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 0.05)
gear_shape.GetCylinderGeometry().rad = 0.1
gear1.AddVisualShape(gear_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
gear2.AddVisualShape(gear_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

# Run the simulation
application.AssetBindAll()
application.AssetUpdateAll()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()