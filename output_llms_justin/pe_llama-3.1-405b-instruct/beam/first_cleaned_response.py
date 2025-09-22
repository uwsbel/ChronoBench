import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


print("Copyright (c) 2017 projectchrono.org")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(1, 0, 0)))


mesh.AddNode(hnode1)
mesh.AddNode(hnode2)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)


msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(0.1, 0.1)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)


belement1.SetSection(msection)


mesh.AddElement(belement1)


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)


vis.SetWindowSize(1024, 768)


vis.SetWindowTitle('Beam Simulation')


vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


vis.AddSkyBox()


vis.AddCamera(chrono.ChVectorD(0, 3, 6))


vis.AddTypicalLights()


while vis.Run():
    
    sys.DoStepDynamics(0.01)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()