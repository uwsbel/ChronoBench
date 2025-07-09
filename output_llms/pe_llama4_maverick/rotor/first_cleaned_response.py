import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))


mesh = fea.ChMesh()


beam_wy = 0.1  
beam_wz = 0.2  
beam_L = 10   
order = 3     


element = fea.ChElementBeamIGA()
element.SetOrder(order)
element.SetNodesCubic()


msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(2.1e11)
msection.SetGshearModulus(8.0769231e10)
msection.SetBeamRaleyghDamping(0.01)


num_nodes = 10
for i in range(num_nodes):
    x = beam_L * i / (num_nodes - 1)
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    mesh.AddNode(node)


for i in range(num_nodes - order):
    element = fea.ChElementBeamIGA()
    element.SetNodes(list(mesh.GetNode(j) for j in range(i, i + order + 1)))
    element.SetSection(msection)
    mesh.AddElement(element)


sys.Add(mesh)


flywheel = chrono.ChBodyEasyCylinder(0.2, 0.5, 7800, True, True, True)
flywheel.SetPos(chrono.ChVector3d(beam_L / 2, 0, 0))
sys.Add(flywheel)


constraint = chrono.ChLinkMateGeneric(True, True, True, True, True, True)
constraint.Initialize(flywheel, mesh.GetNode(int(num_nodes / 2)), False)
sys.Add(constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(mesh.GetNode(0), mesh.GetNode(1), chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_PI))  


sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, 5))


fea.ChVisualizationFEAmesh().SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
fea.ChVisualizationFEAmesh().SetColorscaleMinMax(-0.5, 0.5)
fea.ChVisualizationFEAmesh().SetSmoothFaces(True)
mesh.AddAsset(fea.ChVisualizationFEAmesh())

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)