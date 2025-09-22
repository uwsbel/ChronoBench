import pychrono as chrono
    import pychrono.fea as fea
    import pychrono.irrlicht as chronoirr

    
    sys = chrono.ChSystemSMC()

    
    mesh = fea.ChMesh()

    
    L = 1
    d = 0.01

    
    msection = fea.ChBeamSectionEulerAdvanced()

    
    msection.SetAsCircularSection(d)

    
    msection.SetYoungModulus(0.01e9)
    msection.SetShearModulus(0.01e9 * 0.3)
    msection.SetRayleighDamping(0.000)
    msection.SetCentroid(0)
    msection.SetArea(np.pi * (d ** 2) / 4)
    msection.SetIyy((np.pi * (d ** 4)) / 64)
    msection.SetIzz((np.pi * (d ** 4)) / 64)

    
    builder = fea.ChBuilderCableANCF()

    
    builder.BuildBeam(mesh, msection, 20, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(L, 0, 0))

    
    mtruss = chrono.ChBody()
    mtruss.SetFixed(True)  
    sys.Add(mtruss)  

    
    constraint = fea.ChLinkNodeFrame()
    constraint.Initialize(builder.GetLastNode(), mtruss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0))))
    sys.Add(constraint)  

    
    beam_shape = chrono.ChVisualShapeFEA(mesh)
    beam_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
    beam_shape.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(beam_shape)

    
    section_shape = chrono.ChVisualShapeFEA(mesh)
    section_shape.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    section_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    section_shape.SetZbufferHide(False)
    section_shape.SetSymbolsThickness(0.006)
    section_shape.SetSymbolsScale(0.01)
    section_shape.SetSymbolsTexture(chrono.GetChronoDataFile("textures/symbols_alpha.png"), 12, 12)
    mesh.AddVisualShapeFEA(section_shape)

    
    force_actuator = fea.ChForceActuator()
    force_actuator.PushForce(builder.GetLastNode(), chrono.ChVector3d(0, -0.2, 0))  
    sys.Add(force_actuator)  

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)  
    vis.SetWindowSize(1024, 768)  
    vis.SetWindowTitle('FEA cables')  
    vis.Initialize()  
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
    vis.AddSkyBox()  
    vis.AddCamera(chrono.ChVector3d(0, 1, 4))  
    vis.AddTypicalLights()  

    
    while vis.Run():
        vis.BeginScene()  
        vis.Render()  
        vis.EndScene()  
        sys.DoStepDynamics(0.01)  
print("error happened with only start ```python")