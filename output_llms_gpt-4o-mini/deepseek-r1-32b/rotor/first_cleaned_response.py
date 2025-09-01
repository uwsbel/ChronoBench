importchrono
import pychrono as chrono
import pychrono.fea as fea
import pychrono irrlicht as irr
import numpy as np
import matplotlib.pyplot as plt


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = irr.ChIrrApp(system, "Jeffcott Rotor Simulation", irr.VEC2i(800, 600))
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0))


beam = fea.ChIgaBeam()
beam.SetYoungModulus(2.1e11)  
beam.SetPoissonRatio(0.3)     
beam.SetCrossSection(chrono.ChVectorD(0.01, 0.01, 0.01))  


node1 = fea.ChIgaNode()
node1.SetPos(chrono.ChVectorD(0, 0, 0))
node2 = fea.ChIgaNode()
node2.SetPos(chrono.ChVectorD(1, 0, 0))


beam.AddNode(node1)
beam.AddNode(node2)


element = fea.ChIgaElementBeam()
element.SetNodes(node1, node2)
beam.AddElement(element)


system.Add(beam)


flywheel = chrono.ChBody()
flywheel.SetMass(1.0)  
flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  


cyl = chrono.ChCylinder()
cyl.SetHeight(0.2)
cyl.SetRadius(0.1)
flywheel.AddVisualShape(cyl)


center_node = node1.GetPos() * 0.5 + node2.GetPos() * 0.5
flywheel.SetPos(center_node)
system.Add(flywheel)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(flywheel, beam, chrono.ChCoordsysD(center_node, chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(joint)


motor = chrono.ChRotationalMotor()
motor.SetName("Motor")
motor.SetFrames(beam.GetFrame(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
motor.SetTorque(10.0)  
motor.SetDesiredVelocity(chrono.ChVectorD(0, 0, np.pi))  
system.Add(motor)


visualization = fea.ChIgaFemVisualization()
visualization.SetFem(beam)
application.Add(visualization)


fem_vis = irr.ChIrrNodeFEM()
fem_vis.SetFem(beam)
application.Add(fem_vis)


flywheel_vis = irr.ChIrrNode(flywheel)
application.Add(flywheel_vis)


floor = chrono.ChBody()
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetMass(0)
system.Add(floor)


ground = chrono.ChLinkRigid()
ground.Initialize(floor, beam, chrono.ChCoordsysD(chrono.ChVectorD(0, -1, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(ground)


application.Run()


application.Close()