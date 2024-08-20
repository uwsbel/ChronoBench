import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


chrono.SetChronoDataPath("path/to/chrono/data")


system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


beam = fea.ChMesh()


length = 1.0  
n_elements = 10  
beam_section = fea.ChBeamSectionAdvanced()
beam_section.SetAsRectangularSection(0.01, 0.01)
beam_section.SetYoungModulus(210e9)
beam_section.SetGshearModulus(80.77e9)
beam_section.SetDensity(7800)


nodes = []
for i in range(n_elements + 1):
    x = i * (length / n_elements)
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(x, 0, 0)))
    nodes.append(node)
    beam.AddNode(node)

for i in range(n_elements):
    element = fea.ChElementBeamIGA()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(beam_section)
    beam.AddElement(element)

system.Add(beam)


flywheel = chrono.ChBodyEasyCylinder(0.05, 0.01, 7800)
flywheel.SetPos(chrono.ChVectorD(length / 2, 0, 0))
system.Add(flywheel)


constraint = chrono.ChLinkMateGeneric()
constraint.Initialize(nodes[n_elements // 2], flywheel, False, nodes[n_elements // 2].Frame(), flywheel.GetFrame())
system.Add(constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, nodes[0], chrono.ChFrameD(nodes[0].Frame().GetPos()))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI / 2))  
system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, -1))
vis.AddTypicalLights()


visualize_beam = fea.ChVisualizationFEAmesh(beam)
visualize_beam.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
visualize_beam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
visualize_beam.SetWireframe(True)
beam.AddAsset(visualize_beam)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)