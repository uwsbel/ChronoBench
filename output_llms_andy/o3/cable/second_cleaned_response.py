import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, system, mesh):

        
        
        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)             
        msection_cable.SetYoungModulus(0.01e9)        
        msection_cable.SetRayleighDamping(0.0001)     

        
        
        
        builder = fea.ChBuilderCableANCF()
        builder.BuildBeam(mesh,
                          msection_cable,
                          10,                                   
                          chrono.ChVectorD(0, 0, -0.1),         
                          chrono.ChVectorD(0.5, 0, -0.1))       

        
        
        
        
        front_node = builder.GetLastBeamNodes().front()
        front_node.SetForce(chrono.ChVectorD(0, -0.7, 0))        

        
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        system.AddBody(mtruss)                                   

        hinge = fea.ChLinkNodeFrame()
        hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
        system.Add(hinge)






sys  = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)                                           


visualize_elem = fea.ChVisualShapeFEA(mesh)
visualize_elem.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualize_elem.SetColorscaleMinMax(-0.4, 0.4)
visualize_elem.SetSmoothFaces(True)
visualize_elem.SetWireframe(False)
mesh.AddVisualShapeFEA(visualize_elem)

visualize_nodes = fea.ChVisualShapeFEA(mesh)
visualize_nodes.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualize_nodes.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
visualize_nodes.SetSymbolsThickness(0.006)
visualize_nodes.SetSymbolsScale(0.01)
visualize_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualize_nodes)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))
vis.AddTypicalLights()


solver = chrono.ChSolverMINRES()                       
print("Using MINRES solver")
solver.SetMaxIterations(200)                           
solver.SetTolerance(1e-10)                             
solver.EnableDiagonalPreconditioner(True)              
solver.EnableWarmStart(True)                           
solver.SetVerbose(False)                               
sys.SetSolver(solver)


sys.SetTimestepper(chrono.ChTimestepperEulerImplicitLinearized(sys))


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)