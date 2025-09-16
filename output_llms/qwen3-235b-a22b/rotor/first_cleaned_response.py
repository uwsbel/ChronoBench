import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))


length = 1.0
radius = 0.05
num_elements = 4


nodes = []
for i in range(num_elements + 1):
    x = length * i / num_elements
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(x, 0, 0)))
    nodes.append(node)
    system.Add(node)


mesh = fea.ChMesh()
section = fea.ChBeamSectionIGA()
section.SetYoungModulus(2e11)
section.SetPoissonRatio(0.3)
section.SetDensity(7800)
section.SetArea(chrono.CH_C_PI * radius**2)
section.SetIyy(radius**4 * chrono.CH_C_PI / 4)
section.SetIzz(radius**4 * chrono.CH_C_PI / 4)
section.SetJ(radius**4 * chrono.CH_C_PI / 2)

elements = []
for i in range(num_elements):
    element = fea.ChElementBeamIGA()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(section)
    elements.append(element)
    mesh.AddElement(element)

for node in nodes:
    mesh.AddNode(node)

system.Add(mesh)


flywheel_node = nodes[len(nodes)//2]
flywheel = chrono.ChBody()
flywheel.SetMass(10)
flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
flywheel.SetPos(flywheel_node.GetPos())
flywheel.AddAsset(chrono.ChColorAsset(1, 0, 0))
system.Add(flywheel)

fix_link = chrono.ChLinkMateFix()
fix_link.Initialize(flywheel, flywheel_node)
system.AddLink(fix_link)


motor_body = chrono.ChBody()
motor_body.SetMass(0.1)
motor_body.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
motor_body.SetPos(nodes[0].GetPos())
system.Add(motor_body)

fix_motor_node = chrono.ChLinkMateFix()
fix_motor_node.Initialize(motor_body, nodes[0])
system.AddLink(fix_motor_node)

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(system.GetGroundBody(), motor_body, chrono.ChFrameD(nodes[0].GetPos()))
omega = 10.0
motor.SetSpeedFunction(chrono.ChFunction_Const(omega))
system.AddLink(motor)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor IGA Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0.5, 2), chrono.ChVectorD(0, 0, 0))


mesh_vis = fea.ChVisualizationFEAmesh(mesh)
mesh_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_FEM_DATATYPE_ELEM_STRAIN_ENERGY)
mesh_vis.SetColorscaleMinMax(0.0, 1e4)
mesh.AddAsset(mesh_vis)
mesh.AddAsset(chrono.ChColorAsset(0.6, 0.6, 0.6))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    irrlicht.ChIrrTools.drawAll(vis)
    vis.EndScene()
    system.DoStepDynamics(0.001)