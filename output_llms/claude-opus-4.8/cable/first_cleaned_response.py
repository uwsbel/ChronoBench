import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






sys = chrono.ChSystemSMC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))






mesh = fea.ChMesh()




beam_length = 1.0       
num_elements = 10       
diameter = 0.01         


msection = fea.ChBeamSectionCable()
msection.SetDiameter(diameter)
msection.SetYoungModulus(0.01e9)   
msection.SetRayleighDamping(0.0001)  




builder = fea.ChBuilderCableANCF()



builder.BuildBeam(
    mesh,                               
    msection,                           
    num_elements,                       
    chrono.ChVector3d(0, 0, 0),         
    chrono.ChVector3d(beam_length, 0, 0)  
)


beam_nodes = builder.GetLastBeamNodes()






truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)


first_node = builder.GetLastBeamNodes().front()



hinge_constraint = fea.ChLinkNodeFrame()
hinge_constraint.Initialize(first_node, truss)
sys.Add(hinge_constraint)


sys.Add(mesh)






visual_beam = chrono.ChVisualShapeFEA(mesh)
visual_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visual_beam.SetColorscaleMinMax(-0.4, 0.4)
visual_beam.SetSmoothFaces(True)
visual_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(visual_beam)


visual_nodes = chrono.ChVisualShapeFEA(mesh)
visual_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visual_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visual_nodes.SetSymbolsThickness(0.006)
visual_nodes.SetSymbolsScale(0.01)
visual_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visual_nodes)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ANCF Cable Beam - Hinged End")
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.6, -1.5),
              chrono.ChVector3d(0.5, 0.0, 0.0))
vis.AddTypicalLights()






solver = chrono.ChSolverPardisoMKL()
sys.SetSolver(solver)


stepper = chrono.ChTimestepperHHT(sys)
stepper.SetStepControl(False)
sys.SetTimestepper(stepper)





time_step = 0.01

while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    last_node = beam_nodes.back()
    pos = last_node.GetPos()
    

    vis.EndScene()

    
    sys.DoStepDynamics(time_step)

> solver = chrono.ChSolverMINRES()
> solver.SetMaxIterations(200)
> solver.SetTolerance(1e-10)
> sys.SetSolver(solver)
>