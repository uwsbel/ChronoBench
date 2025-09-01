import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


beam_wy = 0.1  
beam_wz = 0.2  
beam_L = 10   
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)  
msection.SetShearModulus(0.01e9 * 0.3)  
msection.SetRayleighDamping(0.000)  


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)


sys.Add(mesh)


hnode1.SetFixed(True)


myforce1 = chrono.ChForce()
hnode2.AddForce(myforce1)
myforce1.SetF_x(chrono.ChFunction_Const(1000))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Finite Elements Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 3, 3))
vis.AddTypicalLights()


sphere1 = chrono.ChVisualShapeSphere(0.1)
hnode1.AddVisualShape(sphere1)
sphere2 = chrono.ChVisualShapeSphere(0.1)
hnode2.AddVisualShape(sphere2)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)  
    vis.EndScene()