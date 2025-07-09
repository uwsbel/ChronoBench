import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


beam_wy = 0.1  
beam_wz = 0.2  
beam_L = 10   
E = 0.01e9    
nu = 0.3      
G = E / (2 * (1 + nu))  


msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(E)
msection.SetShearModulus(G)
msection.SetRayleighDamping(0.01)


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(beam_L, 0, 0)))
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Finite Element Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 3, 4))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)