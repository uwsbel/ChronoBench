mbs = fea.ChSystemSMC()  
    mbs.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  

    
    flywheel = fea.ChBodyFEA()  
    flywheel.SetFixed(True)  
    flywheel.SetMass(1)  
    flywheel.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  
    mbs.AddBody(flywheel)  

    
    beam = fea.ChBeamIGA()  
    beam.SetChordFunction(chrono.ChFunctionSpline(beam_chord_x, beam_chord_z))  
    beam.SetDiameter(0.1)  
    beam.SetYoungModulus(73.0e9)  
    beam.SetRayleighDamping(0.000)  
    mbs.AddBeam(beam)  

    
    flywheel.SetPos(chrono.ChVector3d(beam.GetCenter().x, beam.GetCenter().y, beam.GetCenter().z))  
    flywheel.AttachNode(beam.GetNodes()[12])  

    
    truss = fea.ChBodyFEA()  
    truss.SetFixed(False)  
    truss.SetMass(0)  
    truss.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  
    mbs.AddBody(truss)  

    
    support = fea.ChLinkNodeFrame()  
    support.Initialize(beam.GetNodes()[0], truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  
    support.SetConstrainedCoords(True, True, True, False, False, False)  
    mbs.AddLink(support)  

    
    motor = fea.ChLinkMotorRotationAngle()  
    motor.Initialize(beam.GetNodes()[13], truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  
    motor.SetAngleFunction(chrono.ChFunctionConst(0))  
    motor.SetMotorFunction(chrono.ChFunctionSine(10))  
    mbs.AddLink(motor)  

    
    mbs.SetSolverType(chrono.ChSolver.Type_PSOR)  
    mbs.SetSolverLinearType(chrono.ChSolver.Type_LU)  

    
    vis = chrono.ChVisualSystemIrrlicht()  
    vis.AttachSystem(mbs)  
    vis.SetWindowSize(1024, 768)  
    vis.SetWindowTitle('Jeffcott rotor with IGA beam')  
    vis.Initialize()  
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
    vis.AddSkyBox()  
    vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam.GetCenter().x, beam.GetCenter().y, beam.GetCenter().z))  
    vis
print("error happened with only start ```python")