import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()


mesh = fea.ChMesh()


material = fea.ChIgaMaterialLinearElastic()
material.E = 210e9  
material.nu = 0.3    
material.rho = 7800  


radius = 0.01  
area = chrono.CH_C_PI * radius**2
Iyy = 0.25 * chrono.CH_C_PI * radius**4  
Izz = Iyy  
J = 0.5 * chrono.CH_C_PI * radius**4     


nodeA = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
nodeC = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0)))
nodeB = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(1, 0, 0)))

mesh.AddNode(nodeA)
mesh.AddNode(nodeC)
mesh.AddNode(nodeB)


element1 = fea.ChIgaElement1D()
element1.SetNodes(nodeA, nodeC)
element1.SetMaterial(material)
element1.SetArea(area)
element1.SetIyy(Iyy)
element1.SetIzz(Izz)
element1.SetJ(J)
mesh.AddElement(element1)

element2 = fea.ChIgaElement1D()
element2.SetNodes(nodeC, nodeB)
element2.SetMaterial(material)
element2.SetArea(area)
element2.SetIyy(Iyy)
element2.SetIzz(Izz)
element2.SetJ(J)
mesh.AddElement(element2)


sys.Add(mesh)


flywheel = chrono.ChBody()
flywheel.SetMass(10)  
flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.2))  
flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))  
sys.Add(flywheel)


link = fea.ChLinkNodeBody()
link.Initialize(nodeC, flywheel)
sys.Add(link)


motor = chrono.ChLinkMotorRotationAngle()

motor_frame = chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 0),
    chrono.Q_from_AngAxis(-chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))
)
motor.Initialize(sys.GetGround(), motor_frame, nodeA, chrono.ChFrameD())


omega = 10.0
ramp = chrono.ChFunction_Ramp(0, omega)
motor.SetAngleFunction(ramp)
sys.Add(motor)


vis = irr.ChIrrApp(sys, 'Jeffcott Rotor IGA Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalLogo()
vis.AddTypicalCamera(irr.vector3df(0, 2, -3))  
vis.AddTypicalLights()


vis.AddMesh(mesh, 'Beam')
vis.GetMeshSceneNode(mesh).SetFEMdataType(chrono.ChIrrApp.FEM_DATA_DISPLACEMENT)
vis.GetMeshSceneNode(mesh).SetColor(chrono.ChColor(0.6, 0.6, 0.6))


vis.AssetBindAll()
vis.AssetUpdateAll()


while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)