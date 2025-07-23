import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso  
import pychrono.irrlicht as chronoirr





class Model1:
    def __init__(self, system, mesh):
        
        msection = fea.ChBeamSectionEulerAdvanced()
        msection.SetDensity(1000)  
        msection.SetYoungModulus( susp_young_modulus)
        msection.SetRayleighDamping(0.000)  
        msection.SetShearCorrectionFactor(1.0)  
        msection.SetSectionWidth(beam_width)  
        msection.SetSectionHeight(beam_height)  
        msection.SetZeroShearCenter()  

        
        
        beam = fea.ChBeamEuler()  
        beam.SetNodes(  
            msection,  
            mesh.LookupNode(0),  
            mesh.LookupNode(5)   
        )
        beam.SetNumberOfLayers(3)  
        beam.SetLayerThickness(0, 0.020)  
        beam.SetLayerThickness(1, 0.020)  
        beam.SetLayerThickness(2, 0.020)  
        beam.SetLayerAngle(1, 45)  
        beam.SetLayerAngle(2, -45)  
        mesh.AddElement(beam)  

        
        
        truss = fea.ChElementBeamUDD()  
        truss.SetNodes(  
            mesh.LookupNode(5),  
            mesh.LookupNode(7)   
        )
        truss.SetSectionDiameter(0.010)  
        mesh.AddElement(truss)  

        
        
        force_vec = chrono.ChVector3d(0, 0.0, -0.2)  
        beam.LookupNode(5).SetForce(beam.LookupNode(5).GetForce() + force_vec)  


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


sys.Add(mesh)


mesh.SetVisible(False)


Model1(sys, mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()


sys.SetSolver(pardiso.ChSolverPardisoMKL())


timesteps = 0.001


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timesteps)