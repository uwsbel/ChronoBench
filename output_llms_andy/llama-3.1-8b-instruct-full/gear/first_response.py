import pychrono as chrono
    import pychrono.irrlicht as chronoirr
    import math

    # Create Chrono physical system
    sys = chrono.ChSystemNSC()

    # Create contact material shared among all bodies for collision detection
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.2)
    mat.SetRestitution(0.01)

    # Create ground body (truss) fixed at origin
    ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, mat)
    ground.SetPos(chrono.ChVector3d(0, 0, -1))  # Position the truss slightly below the origin
    ground.SetFixed(True)  # Fix the truss in place
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(ground)

    # Create rotating bar
    bar = chrono.ChBodyEasyBox(0.2, 2, 0.2, 1000, True, True, mat)
    bar.SetPos(chrono.ChVector3d(1, 0, 0))  # Position the bar at (1, 0, 0)
    bar.SetRot(chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))  # Rotate the bar slightly
    bar.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/metal.jpg"))
    sys.Add(bar)

    # Create motor that drives the rotating bar
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(ground, bar, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    motor.SetSpeedFunction(chrono.ChFunctionConst(0.2))  # Set the rotation speed of the motor
    sys.AddMotor(motor)

    # Create first gear
    gear1 = chrono.ChBodyEasyCylinder(0.4, 0.2, 100, True, True, mat)
    gear1.SetPos(chrono.ChVector3d(1.5, 0, 0))  # Position the first gear at (1.5, 0, 0)
    gear1.SetRot(chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))  # Initialize rotation
    gear1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/gear.jpg"))
    sys.Add(gear1)

    # Add revolute joint between bar and first gear
    joint1 = chrono.ChLinkLockRevolute()
    joint1.Initialize(bar, gear1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    sys.AddLink(joint1)

    # Create second gear
    gear2 = chrono.ChBodyEasyCylinder(0.3, 0.3, 100, True, True, mat)
    gear2.SetPos(chrono.ChVector3d(3, 0, 0))  # Position the second gear at (3, 0, 0)
    gear2.SetRot(chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))  # Initialize rotation
    gear2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/gear.jpg"))
    sys.Add(gear2)

    # Add revolute joint between first gear and second gear
    joint2 = chrono.ChLinkLockRevolute()
    joint2.Initialize(gear1, gear2, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    sys.AddLink(joint2)

    # Create Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)  # Attach Irrlicht to the Chrono system
    vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
    vis.SetWindowTitle('Gears and pulleys')  # Set the window title
    vis.Initialize()  # Initialize the visualization
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
    vis.AddSkyBox()  # Add a skybox for better aesthetics
    vis.AddCamera(chrono.ChVector3d(
print("error happened with only start ```python")