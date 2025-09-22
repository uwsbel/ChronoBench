import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea


step_size = 0



sys = fea.ChSystemNSC()


mesh = fea.ChChordalLineMesh()
mesh.SetName("MyMesh")


n1 = fea.ChFEANode()
n1.SetPosition(chrono.ChVector3d(0, 0, 0))
n2 = fea.ChFEANode()
n2.SetPosition(chrono.ChVector3d(0, 1, 0))
n3 = fea.ChFEANode()
n3.SetPosition(chrono.ChVector3d(1, 1, 0))
n4 = fea.ChFEANode()
n4.SetPosition(chrono.ChVector3d(1, 0, 0))
mesh.AddNode(n1)
mesh.AddNode(n2)
mesh.AddNode(n3)
mesh.AddNode(n4)


beam1 = fea.ChFEABeam()
beam1.SetName("Beam1")
beam1.SetCrossSection(fea.ChFEACrossSectionBox(0.01, 0.01))
beam1.SetMaterial(fea.ChFEMaterialDC(200e9, 0.2))
beam1.SetNodeA(n1)
beam1.SetNodeB(n2)
mesh.AddBeam(beam1)

beam2 = fea.ChFEABeam()
beam2.SetName("Beam2")
beam2.SetCrossSection(fea.ChFEACrossSectionBox(0.01, 0.01))
beam2.SetMaterial(fea.ChFEMaterialDC(200e9, 0.2))
beam2.SetNodeA(n2)
beam2.SetNodeB(n3)
mesh.AddBeam(beam2)

beam3 = fea.ChFEABeam()
beam3.SetName("Beam3")
beam3.SetCrossSection(fea.ChFEACrossSectionBox(0.01, 0.01))
beam3.SetMaterial(fea.ChFEMaterialDC(200e9, 0.2))
beam3.SetNodeA(n3)
beam3.SetNodeB(n4)
mesh.AddBeam(beam3)

beam4 = fea.ChFEABeam()
beam4.SetName("Beam4")
beam4.SetCrossSection(fea.ChFEACrossSectionBox(0.01, 0.01))
beam4.SetMaterial(fea.ChFEMaterialDC(200e9, 0.2))
beam4.SetNodeA(n4)
beam4.SetNodeB(n1)
mesh.AddBeam(beam4)


body = chrono.ChBody()
body.SetName("MyBody")
body.SetMass(0)
body.SetInertiaXX(chrono.ChVector3d(0, 0, 0))
mesh.Initialize(body)


contact_model = fea.ChContactModelChiaia(0.02, 0.02, 0.01, 0.01)
contact_material = fea.ChContactMaterialNSC(contact_model, 1e8, 1e8, 0.01, 0.01)
mesh.SetContactMaterial(contact_material)


force = chrono.ChVector3d(1, 0, 0)
torque = chrono.ChVector3d(0, 0, 0)
mesh.AddForce(fea.ChFEOpForceA(0, n1, force, torque))


vis = fea.ChFEVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('FEA Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5,0.1,0.5))
vis.AddTypicalLights()
vis.AddMeshScene(mesh)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(step_size)