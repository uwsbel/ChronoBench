import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr





class Model1:
    def __init__(self, system, mesh):
        
        msection = fea.ChBeamSectionCable()
        msection.SetDiameter(0.015)           
        msection.SetYoungModulus(0.01e9)      
        msection.SetRayleighDamping(0.0001)   

        
        builder = fea.ChBuilderCableANCF()
        builder.BuildBeam(
            mesh,
            msection,
            10,
            chrono.ChVector3d(0, 0, -0.1),    
            chrono.ChVector3d(0.5, 0, -0.1)   
        )

        
        beam_nodes = builder.GetLastBeamNodes()
        if len(beam_nodes) > 0:
            beam_nodes[0].SetForce(chrono.ChVector3d(0, -0.7, 0))  

        
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        system.Add(mtruss)

        
        constraint = fea.ChLinkNodeFrame()
        constraint.Initialize(beam_nodes[-1], mtruss)
        system.Add(constraint)






sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
model = Model1(sys, mesh)
sys.Add(mesh)


vis_mz = chrono.ChVisualShapeFEA(mesh)
vis_mz.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_mz.SetColorscaleMinMax(-0.4, 0.4)
vis_mz.SetSmoothFaces(True)
vis_mz.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_mz)


vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()


solver = chrono.ChSolverMINRES()
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
solver.SetMaxIterations(200)                     
solver.SetTolerance(1e-10)                       
solver.EnableDiagonalPreconditioner(True)        
solver.EnableWarmStart(True)                     
solver.SetVerbose(False)                         
sys.SetSolver(solver)


ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)