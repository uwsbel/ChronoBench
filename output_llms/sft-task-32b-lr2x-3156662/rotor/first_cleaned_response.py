import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


beam_length = 1.0
num_elements = 10
beam = fea.ChIgaBeamFEA()
beam.SetName("Jeffcott Rotor Beam")
system.Add(beam)

nodes = []
for i in range(num_elements + 1):
    x = i * beam_length / num_elements
    node = fea.ChNodeFEAxyz(chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    beam.AddNode(node)
    nodes.append(node)

elements = []
for i in range(num_elements):
    element = fea.ChElementIgaBeam()
    element.SetNodes(nodes[i], nodes[i + 1])
    section = fea.ChIgaSectionBeam()
    section.SetAsRectangularSection(0.01, 0.01)  
    section.SetYoungModulus(210e9)
    section.SetDensity(7800)
    element.SetSection(section)
    beam.AddElement(element)
    elements.append(element)


flywheel = chrono.ChBody()
flywheel.SetName("Flywheel")
flywheel.SetMass(1.0)
flywheel.SetInertiaXX(chrono.ChVector3d(0.00125, 0.00125, 0.0025))
flywheel.SetPos(chrono.ChVector3d(beam_length / 2, 0, 0))
system.AddBody(flywheel)


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.SetName("Flywheel-BEAM Revolute Joint")
rev_joint.Initialize(flywheel, beam, chrono.ChFramed(chrono.ChVector3d(beam_length / 2, 0, 0), chrono.QUNIT))
system.AddLink(rev_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.SetName("Beam Rotational Motor")
motor.Initialize(system.GetGroundBody(), beam, chrono.ChFramed(chrono.ChVector3d(beam_length, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(100.0))  
system.AddLink(motor)


beam_vis = chrono.ChVisualShapeFEA()
beam_vis.SetFEMesh(beam.GetMesh())
beam_vis.SetDrawMode(chrono.ChVisualShapeFEA.DrawMode.ELEMENTS)
beam.AddVisualShape(beam_vis)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Jeffcott Rotor with IGA Beam')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)