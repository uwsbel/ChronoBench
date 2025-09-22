import pychrono as chrono
    import pychrono.fea as fea
    import pychrono.irrlicht as chronoirr

    
    sys = chrono.ChSystemSMC()

    
    mesh = fea.ChMesh()
    mesh.SetRefFrame(chrono.ChFrame())
    sys.Add(mesh)

    
    hnode1 = fea.ChNodeFEAxyz(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    hnode2 = fea.ChNodeFEAxyz(chrono.ChFramed(chrono.ChVector3d(2, 0, 0)))
    hnode3 = fea.ChNodeFEAxyz(chrono.ChFramed(chrono.ChVector3d(2, 2, 0)))
    hnode4 = fea.ChNodeFEAxyz(chrono.ChFramed(chrono.ChVector3d(0, 2, 0)))
    mesh.AddNode(hnode1)
    mesh.AddNode(hnode2)
    mesh.AddNode(hnode3)
    mesh.AddNode(hnode4)

    
    h elem1 = fea.ChElementBeamEulerAdvanced()
    h elem1.SetNodes(hnode1, hnode2)
    h elem1.SetYoungModulus(200e9)
    h elem1.SetShearModulus(0.1e9)
    h elem1.SetRayleighDamping(0.000)
    mesh.AddElement(elem1)

    h elem2 = fea.ChElementBeamEulerAdvanced()
    h elem2.SetNodes(hnode2, hnode3)
    h elem2.SetYoungModulus(200e9)
    h elem2.SetShearModulus(0.1e9)
    h elem2.SetRayleighDamping(0.000)
    mesh.AddElement(elem2)

    h elem3 = fea.ChElementBeamEulerAdvanced()
    h elem3.SetNodes(hnode3, hnode4)
    h elem3.SetYoungModulus(200e9)
    h elem3.SetShearModulus(0.1e9)
    h elem3.SetRayleighDamping(0.000)
    mesh.AddElement(elem3)

    h elem4 = fea.ChElementBeamEulerAdvanced()
    h elem4.SetNodes(hnode4, hnode1)
    h elem4.SetYoungModulus(200e9)
    h elem4.SetShearModulus(0.1e9)
    h elem4.SetRayleighDamping(0.000)
    mesh.AddElement(elem4)

    
    mvisualshape1 = chrono.ChVisualShapeFEA(mesh)
    mvisualshape1.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
    mvisualshape1.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(mvisualshape1)

    mvisualshape2 = chrono.ChVisualShapeFEA(mesh)
    mvisualshape2.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    mvisualshape2.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    mvisualshape2.SetSymbolsThickness(0.006)
    mvisualshape2.SetSymbolsScale(0.01)
    mesh.AddVisualShapeFEA(mvisualshape2)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('FEA test: beam in x direction')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(1, 1, 0))
    vis.AddTypicalLights()

    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(0.001)