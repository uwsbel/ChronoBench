import pychrono as chrono
import pychrono.igabase as igabase
import pychrono.postprocess as postprocess
import pychrono.irrlicht as irr

def main():
    # Initialize the Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # Create ground body
    ground = chrono.ChBody()
    ground.SetIdentifier(-1)
    ground.SetBodyFixed(True)
    ground.SetCollide(False)
    system.Add(ground)

    # Define beam geometry using NURBS curve
    control_points = [
        chrono.ChVectorD(0, 0, 0),
        chrono.ChVectorD(0.5, 0, 0),
        chrono.ChVectorD(1, 0, 0)
    ]
    knots = [0, 0, 0, 1, 1, 1]
    weights = [1, 1, 1]

    curve = chrono.ChNurbsCurve()
    curve.SetDegree(2)
    curve.SetControlPoints(control_points)
    curve.SetKnots(knots)
    curve.SetWeights(weights)

    # Beam cross-section properties
    section = chrono.ChBeamSectionIga()
    section.SetMaterialYoungModulus(2.1e11)  # Steel-like
    section.SetMaterialPoissonRatio(0.3)
    section.SetArea(0.01)  # Cross-sectional area
    section.SetIy(0.0001)  # Moment of inertia around Y
    section.SetIz(0.0001)  # Moment of inertia around Z
    section.SetJ(0.0002)   # Torsional inertia

    # Create IGA beam
    beam = chrono.ChBeamIga()
    beam.Initialize(curve, section, 1)  # 1 element for simplicity
    system.Add(beam)

    # Rotational motor setup (drive beam end around Y-axis)
    rot = chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0))
    coordsys = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), rot)
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(beam, ground, coordsys)
    motor.SetMotorMode(chrono.ChLinkMotorRotationSpeed.MOTOR_MODE_SPEED)
    motor.SetSpeed(chrono.ChFunction.ConstantFunction(10))  # 10 rad/s
    system.Add(motor)

    # Flywheel at beam's center
    flywheel = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
    flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))
    system.Add(flywheel)

    # Attach flywheel to beam's midpoint
    joint_fly = chrono.ChLinkLockPointFrame()
    joint_fly.Initialize(flywheel, beam, chrono.ChVectorD(0.5, 0, 0))
    system.Add(joint_fly)

    # Visualization setup
    application = irr.ChIrrApp(system, 'Jeffcott Rotor Simulation', irr.dimension2du(800, 600))
    application.AddTypicalLights()
    application.AddTypicalCamera(chrono.ChVectorD(0.5, 1, 0.5), chrono.ChVectorD(0.5, 0, 0))
    application.AddAllTypicalEventHandlers()

    # FEM visualization
    post = postprocess.ChPostProcess()
    post.AddFEMMesh(beam)
    application.SetPostProcess(post)

    # Update visualization assets
    application.AssetBindAll()
    application.AssetUpdateAll()

    # Simulation loop
    while application.GetDevice().run():
        system.DoStepDynamics(1e-3)
        application.BeginScene()
        application.DrawAll()
        application.EndScene()

if __name__ == '__main__':
    main()