import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


beam_L = 1.0  
beam_wy = 0.05  
beam_wz = 0.05  
young_modulus = 2.0e11  
shear_modulus = 8.0e10  
density = 7800  
damping = 0.001 


mesh = fea.ChMesh()
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(young_modulus)
msection.SetShearModulus(shear_modulus)
msection.SetDensity(density)
msection.SetRayleighDamping(damping)


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)


body = fea.ChBodyEasyMesh(mesh)
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetMass(1.0)  
body.SetInertiaXX(chrono.ChVector3d(1,1,1))
sys.Add(body)


body.SetFixed(True)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -5))
vis.AddTypicalLights()


body.AddVisualShape(chrono.ChVisualShapeMesh(mesh))


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)