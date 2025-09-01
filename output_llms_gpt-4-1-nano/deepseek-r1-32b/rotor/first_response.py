importchrono
import pychrono as chrono
import pychrono.fea as fea
import pychrono irrlicht as irr
import numpy as np
import matplotlib.pyplot as plt

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create an Irrlicht application for visualization
application = irr.ChIrrApp(system, "Jeffcott Rotor Simulation", irr.VEC2i(800, 600))
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0))

# Create the beam
beam = fea.ChIgaBeam()
beam.SetYoungModulus(2.1e11)  # Young's modulus
beam.SetPoissonRatio(0.3)     # Poisson's ratio
beam.SetCrossSection(chrono.ChVectorD(0.01, 0.01, 0.01))  # Cross-sectional dimensions

# Define beam nodes
node1 = fea.ChIgaNode()
node1.SetPos(chrono.ChVectorD(0, 0, 0))
node2 = fea.ChIgaNode()
node2.SetPos(chrono.ChVectorD(1, 0, 0))

# Add nodes to the beam
beam.AddNode(node1)
beam.AddNode(node2)

# Create and add the IGA beam element
element = fea.ChIgaElementBeam()
element.SetNodes(node1, node2)
beam.AddElement(element)

# Add the beam to the system
system.Add(beam)

# Create the flywheel
flywheel = chrono.ChBody()
flywheel.SetMass(1.0)  # Mass of the flywheel
flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  # Moment of inertia

# Create a cylindrical shape for the flywheel
cyl = chrono.ChCylinder()
cyl.SetHeight(0.2)
cyl.SetRadius(0.1)
flywheel.AddVisualShape(cyl)

# Attach the flywheel to the center of the beam
center_node = node1.GetPos() * 0.5 + node2.GetPos() * 0.5
flywheel.SetPos(center_node)
system.Add(flywheel)

# Create a revolute joint to attach the flywheel to the beam
joint = chrono.ChLinkLockRevolute()
joint.Initialize(flywheel, beam, chrono.ChCoordsysD(center_node, chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(joint)

# Create a rotational motor to drive the beam
motor = chrono.ChRotationalMotor()
motor.SetName("Motor")
motor.SetFrames(beam.GetFrame(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
motor.SetTorque(10.0)  # Torque applied by the motor
motor.SetDesiredVelocity(chrono.ChVectorD(0, 0, np.pi))  # Desired angular velocity
system.Add(motor)

# Add visualization elements
visualization = fea.ChIgaFemVisualization()
visualization.SetFem(beam)
application.Add(visualization)

# Create a FEM visualization node
fem_vis = irr.ChIrrNodeFEM()
fem_vis.SetFem(beam)
application.Add(fem_vis)

# Create a visualization node for the flywheel
flywheel_vis = irr.ChIrrNode(flywheel)
application.Add(flywheel_vis)

# Add a floor to prevent objects from falling
floor = chrono.ChBody()
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetMass(0)
system.Add(floor)

# Create a ground constraint for the floor
ground = chrono.ChLinkRigid()
ground.Initialize(floor, beam, chrono.ChCoordsysD(chrono.ChVectorD(0, -1, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(ground)

# Run the simulation
application.Run()

# Close the application
application.Close()