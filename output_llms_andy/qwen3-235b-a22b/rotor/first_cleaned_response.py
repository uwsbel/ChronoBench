import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math


sys = chrono.ChSystemNSC()


beam_length = 1.0
r = 0.05  
E = 210e9  
nu = 0.3  
G = E / (2 * (1 + nu))  
rho = 7800  


area = math.pi * r**2
Iyy = math.pi * r**4 / 4
Izz = Iyy
J = math.pi * r**4 / 2  


material = fea.ChMaterialBeamIGA()
material.E = E
material.G = G
material.rho = rho


mesh = fea.ChMesh()


node_start = fea.ChNodeFEAxyz(chrono.ChVectorD(0, 0, 0))
node_center = fea.ChNodeFEAxyz(chrono.ChVectorD(beam_length/2, 0, 0))
node_end = fea.ChNodeFEAxyz(chrono.ChVectorD(beam_length, 0, 0))
mesh.AddNode(node_start)
mesh.AddNode(node_center)
mesh.AddNode(node_end)


def create_iga_element(nodeA, nodeB):
    element = fea.ChIgaElementBeam()
    element.SetNodes(nodeA, nodeB)
    element.SetMaterial(material)
    element.SetArea(area)
    element.SetIyy(Iyy)
    element.SetIzz(Izz)
    element.SetJ(J)
    return element

mesh.AddElement(create_iga_element(node_start, node_center))
mesh.AddElement(create_iga_element(node_center, node_end))

sys.Add(mesh)


flywheel_mass = 5.0
flywheel_radius = 0.15
flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVectorD(
    0.5 * flywheel_mass * flywheel_radius**2,  
    0.25 * flywheel_mass * flywheel_radius**2,  
    0.25 * flywheel_mass * flywheel_radius**2   
))
flywheel.SetPos(node_center.GetPos())
sys.Add(flywheel)


link_flywheel = fea.ChLinkNodeBody()
link_flywheel.Initialize(node_center, flywheel)
sys.Add(link_flywheel)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


start_body = chrono.ChBody()
start_body.SetPos(node_start.GetPos())
sys.Add(start_body)

link_start = fea.ChLinkNodeBody()
link_start.Initialize(node_start, start_body)
sys.Add(link_start)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(ground, start_body, chrono.ChFrameD(node_start.GetPos()))
sys.Add(motor)


omega = 20
ramp_function = chrono.ChFunction_Ramp(0, omega)
motor.SetAngleFunction(ramp_function)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor IGA Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 1, 3), chrono.ChVectorD(0, 0, 0))


fea_vis = irr.ChVisualShapeFEA(mesh)
fea_vis.SetFEMdataType(irr.ChVisualShapeFEA.DataType_SURFACE)
fea_vis.SetWireframe(False)
mesh.AddVisualShapeFEA(fea_vis)


flywheel_vis = chrono.ChVisualShapeSphere(flywheel_radius)
flywheel_vis.SetColor(chrono.ChColor(1, 0, 0))
flywheel.AddVisualShape(flywheel_vis)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)