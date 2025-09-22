import pychrono.core as chrono
    import pychrono.fea as fea
    import pychrono.irrlicht as chronoirr
    import pychrono.pardisomkl as mkl
    import numpy as np

    
    sys = chrono.ChSystemSMC()

    
    mesh = fea.ChMesh()

    
    msection = fea.ChBeamShellSectionKirchhoffIsotropo()

    
    msection.SetThickness(0.006)

    
    msection.SetDensity(1000)

    
    msection.SetYoungModulus(6e4)

    
    msection.SetPoissonRatio(0.0)

    
    mesh.AssignSection(msection)

    
    npts = 20
    for i in range(npts + 1):
        for j in range(npts + 1):
            p = chrono.ChVector3d(i * 0.1, j * 0.1, 0.0)
            
            node = fea.ChNodeFEAxyz(p)
            mesh.AddNode(node)

    
    for i in range(npts):
        for j in range(npts):
            me = fea.ChElementShellKirchhoff()
            boundary_1 = mesh.GetNode((i + 1, j))
            boundary_2 = mesh.GetNode((i, j))
            boundary_3 = mesh.GetNode((i, j + 1))
            boundary_4 = mesh.GetNode((i + 1, j + 1))
            me.SetNodes(boundary_1, boundary_2, boundary_3, boundary_4)
            mesh.AddElement(me)

    
    sys.Add(mesh)

    
    truss = chrono.ChBody()
    truss.SetFixed(True)
    sys.Add(truss)

    
    constraint = chrono.ChLinkMateGeneric()
    constraint.Initialize(mesh, truss, False, mesh.GetFrameAt(0, 0, 0), truss.GetFrame())
    sys.Add(constraint)

    
    driver = chrono.ChLinkMotorRotationSpeed()

    
    driver.Initialize(mesh.GetNode(0, 0), truss, False, mesh.GetFrameAt(0, 0, 0), truss.GetFrame())

    
    driver.SetSpeedFunction(chrono.ChFunctionConst(0.1))

    
    sys.Add(driver)

    
    force = chrono.ChLinkForce()

    
    force.Initialize(mesh.GetNode(npts, npts), None, False, mesh.GetFrameAt(npts, npts, 0), chrono.ChVector3d(0, 0, -1))

    
    force.SetForceFunction(chrono.ChFunctionConst(chrono.ChVector3d(-0.1, 0, 0)))

    
    sys.Add(force)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('FEA mesh with PardisoMKL solver')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.0))
    vis.AddTypicalLights()

    
    mkl_solver = mkl.ChSolverPardisoMKL()
    sys.SetSolver(mkl_solver)

    
    while vis.Run():
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
print("error happened with only start ```python")