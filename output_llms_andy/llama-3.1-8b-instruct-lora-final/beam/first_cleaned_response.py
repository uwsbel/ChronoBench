import pychrono as ch
    import pychrono.fea as fea
    import pychrono.irrlicht as chrig
    import math

    
    nbelements = 20

    
    L = 1.0

    
    n_nodes = nbelements + 1

    
    mesh_width = nbelements * L
    mesh_height = 0.2

    
    beam_wy = 0.1  
    beam_wz = 0.1  
    beam_rho = 2700  
    beam_E = 73e9  
    beam_ro = beam_E / (2 * math.pi)

    
    mesh_grid_sp = L / nbelements

    
    sys = ch.ChSystemNSC()

    
    mesh = fea.ChMesh()

    
    mesh_body = ch.ChBody()
    mesh_body.SetMesh(mesh)
    mesh_body.SetFixed(False)
    mesh_body.SetMassless(True)
    sys.Add(mesh_body)

    
    nodes = []
    for i in range(n_nodes):
        node = fea.ChNodeFEA()
        mesh.AddNode(node)
        nodes.append(node)

    
    for i in range(nbelements):
        start_node = nodes[i]
        end_node = nodes[i + 1]
        beam = fea.ChElementBeamGNSA()
        beam.SetNodes(beam, start_node, end_node)
        beam.SetSectionProperties(beam, fea.ChBeamSectionEulerAdvanced())
        beam.SetMaterialProperties(beam, fea.ChBeamMaterialSMC())
        mesh.AddElement(beam)

    
    boxcoll = ch.ChBoxCollisionShape(mesh_width, mesh_height, mesh_height)
    boxcoll.SetOffset(ch.ChVector3d(mesh_width / 2, mesh_height / 2, 0))
    mesh_body.AddCollisionShape(boxcoll)

    
    visualizemesh = ch.ChVisualShapeFEA(mesh)
    visualizemesh.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
    visualizemesh.SetColor(ch.ChColor(0.6, 0.1, 0.4))
    mesh_body.AddVisualShape(visualizemesh)

    
    visualizenode = ch.ChVisualShapeFEA(mesh)
    visualizenode.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    visualizenode.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
    visualizenode.SetZbufferHide(False)
    visualizenode.SetSymbolsThickness(0.006)
    visualizenode.SetSymbolsScale(0.01)
    mesh_body.AddVisualShape(visualizenode)

    
    vis = ch.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('FEA beams')
    vis.Initialize()
    vis.AddLogo(ch.ChLogoType_NONE)
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(0, 0.6, 1.5))
    vis.AddTypicalLights()

    
    solver = ch.ChSolverPardiso()
    sys.SetSolver(solver)

    
    timestepper = ch.ChTimestepperEulerImplicitLinearized(sys)
    timestepper.SetStepControl(ch.ChStepControl(0.001, 0.001, 0.001))

    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(0.01)