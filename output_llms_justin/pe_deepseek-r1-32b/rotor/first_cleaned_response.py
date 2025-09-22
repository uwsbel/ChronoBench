import math
import numpy as np
import pychrono as chrono
import pychrono.fea as fea
import pychrono irrlicht as chronoirr


sys = chrono.ChSystemSMC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()


beam_length = 2.0
num_elements = 10
beam_width = 0.1
beam_height = 0.1
young_modulus = 2.0e11
shear_modulus = 8.0e10
density = 8000.0
damping = 0.001
flywheel_mass = 100.0
flywheel_inertia = 0.1


beam = fea.ChBeam()
beam.SetYoungModulus(young_modulus)
beam.SetShearModulus(shear_modulus)
beam.SetDensity(density)
beam.SetDamping(damping)


nodes = []
for i in range(num_elements + 1):
    x = i * (beam_length / num_elements)
    node = fea.ChNodeFEAxyzrot()
    node.SetPos(chrono.ChVector3d(x, 0, 0))
    sys.Add(node)
    nodes.append(node)


for i in range(num_elements):
    element = fea.ChElementBeam()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(fea.ChBeamSectionEuler())
    element.GetSection().SetAsRectangularSection(beam_width, beam_height)
    sys.Add(element)


center_node = nodes[num_elements // 2]
flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVector3d(flywheel_inertia, flywheel_inertia, flywheel_inertia))
flywheel.SetPos(center_node.GetPos())
sys.Add(flywheel)


flywheel_shape = chrono.ChVisualShapeCylinder(0.2, 0.4)
flywheel_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
flywheel.AddVisualShape(flywheel_shape)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(nodes[0], sys.GetGround(), chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.Add(motor)


motor.SetSpeedFunction(chrono.ChFunction_Const(5.0))  


beam_mesh = fea.ChFEMMesh()
beam_mesh.AddNodes(nodes)
beam_mesh.AddElements(elements)
sys.Add(beam_mesh)


while vis.Run():
    sys.DoStepDynamics(0.001)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()