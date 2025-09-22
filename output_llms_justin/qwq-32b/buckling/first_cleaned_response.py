import pychrono as chrono
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetMaxItersSolverSpeed(100)
sys.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


sys.SetTimestepper(chrono.ChTimestepperHHT(sys))
sys.GetTimestepper().SetAlpha(-0.2)

vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling FEA Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSky()
vis.AddGrid()
sys.SetVisualSystem(vis)


beam_length = 2.0
num_elements = 20
element_length = beam_length / num_elements

cross_section_width = 0.1
cross_section_height = 0.02
material_density = 7850
young_modulus = 2.1e11
poisson_ratio = 0.3


nodes = []
for i in range(num_elements + 1):
    x = i * element_length
    node = chrono.ChNodeFEAbeam()
    node.SetPos(chrono.ChVectorD(x, 0, 0))
    sys.AddNode(node)
    nodes.append(node)


elements = []
for i in range(num_elements):
    n1 = nodes[i]
    n2 = nodes[i + 1]
    elem = chrono.ChElementBeam()
    elem.SetNodes(n1, n2)
    elem.Set_E(young_modulus)
    elem.Set_G(young_modulus / (2 * (1 + poisson_ratio)))
    elem.Set_density(material_density)
    area = cross_section_width * cross_section_height
    elem.Set_Area(area)
    Iz = (cross_section_width * cross_section_height**3) / 12
    elem.Set_Iz(Iz)
    sys.AddElement(elem)
    elements.append(elem)


fixed = chrono.ChLinkLockPointFrame()
fixed.Initialize(nodes[0].GetBody(), sys.Get_ground(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
sys.Add(fixed)


target_body = chrono.ChBody()
target_body.SetBodyFixed(True)
target_body.SetPos(nodes[-1].GetPos())
sys.Add(target_body)

prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(nodes[-1].GetBody(), target_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
sys.Add(prismatic)

motor = chrono.ChLinkMotorLinear()
motor.Initialize(nodes[-1].GetBody(), target_body, chrono.VECT_X)
motor.SetMode(chrono.ChLinkMotorLinear.Modes.POSITION)
motor.SetReferencePositionFunction(lambda time: -0.1 * time)  
sys.Add(motor)


for node in nodes:
    sphere = chrono.ChSphereShape()
    sphere.GetSphereGeometry().rad = 0.02
    sphere.SetColor(chrono.ChColor(0, 0, 1))
    node.AddAsset(sphere)

for elem in elements:
    n1 = elem.GetNodeA()
    n2 = elem.GetNodeB()
    line = chrono.ChLineShape()
    line.SetPoints(n1.GetPos(), n2.GetPos())
    line.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    line.SetThickness(0.005)
    elem.AddAsset(line)


simulation_time = 10.0
time_step = 0.001

while vis.Run() and sys.GetChTime() < simulation_time:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()