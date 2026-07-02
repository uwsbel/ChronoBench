import pychrono as chrono
from pychrono import fea, irrlicht


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


mesh = fea.ChMesh()


section = fea.ChContinuumCableSectionANCF()
section.SetRadius(0.05)  
section.SetYoungModulus(1e6)  
section.SetPoissonRatio(0.3)  
section.SetDamping(0.01)  


builder = fea.ChBuilderCableANCF()
builder.BuildBeam(
    mesh, 
    section, 
    20,  
    chrono.ChVectorD(0, 0, 0),  
    chrono.ChVectorD(5, 0, 0),  
    0  
)


if builder.GetNodes():
    builder.GetNodes()[0].SetFixed(True)


for element in mesh.GetElements():
    vis = fea.ChVisualizationFEAElementBeam()
    vis.SetColorscale(False)
    vis.SetDefaultColor(chrono.ChColor(0.6, 0.6, 0.6))
    element.AddAsset(vis)


for node in mesh.GetNodes():
    sphere = chrono.ChVisualShapeSphere(0.05)
    sphere.SetColor(chrono.ChColor(0.2, 0.2, 1.0))
    node.AddVisualShape(sphere)


sys.Add(mesh)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 5), chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChVectorD(10, 10, 10), chrono.ChColor(1, 1, 1), 100)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()