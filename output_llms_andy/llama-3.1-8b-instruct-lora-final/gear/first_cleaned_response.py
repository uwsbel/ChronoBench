import pychrono as ch
    import pychrono.irrlicht as chbrand

    
    sys = ch.ChSystemNSC()

    
    g = -10

    
    sphere_mat = ch.ChContactMaterialNSC()
    sphere_mat.SetFriction(0.5)
    sphere = ch.ChBodyEasySphere(2.5, 1800, True, True, sphere_mat)
    sphere.SetPos(ch.ChVector3d(0, 0, 1))
    sphere.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    sphere.SetGravitationalAcceleration(ch.ChVector3d(0, 0, g))
    sys.Add(sphere)

    
    box_mat = ch.ChContactMaterialNSC()
    box_mat.SetFriction(0.1)
    box = ch.ChBody()
    box.SetPos(ch.ChVector3d(-2, 0, 1))
    box.SetFixed(False)
    box.SetMass(1000)
    box.SetInertiaXX(ch.ChVector3d(1000, 1000, 1000))
    box.SetGravitationalAcceleration(ch.ChVector3d(0, 0, g))
    box.AddVisualShape(ch.ChVisualShapeBox(1.2, 0.2, 0.2))
    sys.Add(box)

    
    truss = ch.ChBody()
    truss.SetPos(ch.ChVector3d(-1, 0, 1))
    truss.SetFixed(True)
    truss.SetGravitationalAcceleration(ch.ChVector3d(0, 0, g))
    sys.Add(truss)

    
    bar = ch.ChBody()
    bar.SetPos(ch.ChVector3d(0, 0, 1))
    bar.SetFixed(False)
    bar.SetMass(100)
    bar.SetInertiaXX(ch.ChVector3d(100, 100, 100))
    bar.SetGravitationalAcceleration(ch.ChVector3d(0, 0, g))
    sys.Add(bar)

    
    gear = ch.ChLinkMateGeneric()
    gear.Initialize(sphere, bar, False, False, False, False)
    gear.SetConstrainedCoords(False, False, False, False, False, False, False, False, False, False, False, False)
    gear.SetLinkFrame1(ch.ChFramed(sphere.Frame()))
    gear.SetLinkFrame2(ch.ChFramed(bar.Frame()))
    sys.Add(gear)

    
    motor = ch.ChLinkMotorRotationSpeed()
    motor.Initialize(sphere, bar, False, False, False, False)
    motor.SetConstrainedCoords(False, False, False, False, False, False, False, False, False, False, False, False)
    motor.SetMotorFunction(ch.ChFunctionSine(2 * ch.CH_PI))
    sys.Add(motor)

    
    gear = ch.ChLinkMateGeneric()
    gear.Initialize(bar, box, False, False, False, False)
    gear.SetConstrainedCoords(False, False, False, False, False, False, False, False, False, False, False, False)
    gear.SetLinkFrame1(ch.ChFramed(bar.Frame()))
    gear.SetLinkFrame2(ch.ChFramed(box.Frame()))
    sys.Add(gear)

    
    gear = ch.ChLinkMateGeneric()
    gear.Initialize(box, truss, False, False, False, False)
    gear.SetConstrainedCoords(False, False, False, False, False, False, False, False, False, False, False, False)
    gear.SetLinkFrame1(ch.ChFramed(box.Frame()))
    gear.SetLinkFrame2(ch.ChFramed(truss.Frame()))
    sys.Add(gear)

    
    gear = ch.ChLinkMateGeneric()
    gear.Initialize(truss, sphere, False, False, False, False)
    gear.SetConstrainedCoords(False, False, False, False, False, False, False, False, False, False, False, False)
    gear.SetLinkFrame1(ch.ChFramed(truss.Frame()))
    gear.SetLinkFrame2(ch.ChFramed(sphere.Frame()))
    sys.Add(gear)

    
print("error happened with only start ```python")