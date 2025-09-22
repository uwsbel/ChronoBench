# Create the Jeffcott rotor system
    mbs = fea.ChSystemSMC()  # Initialize the mechanical system
    mbs.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Set gravity

    # Create the flywheel and attach it to the ground
    flywheel = fea.ChBodyFEA()  # Create a FEA body for the flywheel
    flywheel.SetFixed(True)  # Fix the flywheel in space
    flywheel.SetMass(1)  # Set the mass of the flywheel
    flywheel.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set inertia tensor
    mbs.AddBody(flywheel)  # Add the flywheel to the system

    # Create the Jeffcott rotor as an IGA beam
    beam = fea.ChBeamIGA()  # Create the IGA beam
    beam.SetChordFunction(chrono.ChFunctionSpline(beam_chord_x, beam_chord_z))  # Set the chord function
    beam.SetDiameter(0.1)  # Set the diameter of the beam
    beam.SetYoungModulus(73.0e9)  # Set Young's modulus for the beam material
    beam.SetRayleighDamping(0.000)  # Set Rayleigh damping for the beam
    mbs.AddBeam(beam)  # Add the beam to the system

    # Attach the flywheel to the center of the beam
    flywheel.SetPos(chrono.ChVector3d(beam.GetCenter().x, beam.GetCenter().y, beam.GetCenter().z))  # Position the flywheel
    flywheel.AttachNode(beam.GetNodes()[12])  # Attach the flywheel to the 13th node of the beam

    # Create a truss body
    truss = fea.ChBodyFEA()  # Create a FEA body for the truss
    truss.SetFixed(False)  # Fix the truss in space
    truss.SetMass(0)  # Set the mass of the truss (zero for a fixed truss)
    truss.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  # Set inertia tensor (zero for a fixed truss)
    mbs.AddBody(truss)  # Add the truss to the system

    # Create a support and attach the truss to the ground
    support = fea.ChLinkNodeFrame()  # Create a node-frame link for the support
    support.Initialize(beam.GetNodes()[0], truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # Initialize the support
    support.SetConstrainedCoords(True, True, True, False, False, False)  # Constrain x, y, z translations and rotations
    mbs.AddLink(support)  # Add the support to the system

    # Create a rotational motor
    motor = fea.ChLinkMotorRotationAngle()  # Create a rotational motor
    motor.Initialize(beam.GetNodes()[13], truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # Initialize the motor
    motor.SetAngleFunction(chrono.ChFunctionConst(0))  # Set the initial angle to zero
    motor.SetMotorFunction(chrono.ChFunctionSine(10))  # Set the angle function to a sine wave
    mbs.AddLink(motor)  # Add the motor to the system

    # Set the solver type
    mbs.SetSolverType(chrono.ChSolver.Type_PSOR)  # Set the solver to PSOR
    mbs.SetSolverLinearType(chrono.ChSolver.Type_LU)  # Set the linear solver to LU

    # Create the visual system
    vis = chrono.ChVisualSystemIrrlicht()  # Create the Irrlicht visual system
    vis.AttachSystem(mbs)  # Attach the simulation system to the visual system
    vis.SetWindowSize(1024, 768)  # Set the window size
    vis.SetWindowTitle('Jeffcott rotor with IGA beam')  # Set the window title
    vis.Initialize()  # Initialize the visual system
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
    vis.AddSkyBox()  # Add a skybox
    vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam.GetCenter().x, beam.GetCenter().y, beam.GetCenter().z))  # Add a camera
    vis
print("error happened with only start ```python")