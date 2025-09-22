import pychrono as ch
    import pychrono.irrlicht as chrig
    import math

    
    sys = ch.ChSystemNSC()

    
    g = -9.81

    
    sphere_mat = ch.ChContactMaterialNSC()
    sphere_mat.SetFriction(0.5)
    sphere = ch.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
    sphere.SetPos(ch.ChVector3d(1, 0.5, 0))
    sphere.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    sphere.SetGravitationalAcceleration(ch.ChVector3d(0, g, 0))
    sphere.SetName("sphere")
    sys.Add(sphere)

    
    box_mat = ch.ChContactMaterialNSC()
    box_mat.SetFriction(0.1)
    box = ch.ChBodyEasyBox(1.1, 0.2, 0.3, 1850, True, True, box_mat)
    box.SetPos(ch.ChVector3d(-1, 0.5, 0))
    box.SetGravitationalAcceleration(ch.ChVector3d(0, g, 0))
    box.SetName("box")
    sys.Add(box)

    
    truss = ch.ChBodyEasyBox(10, 1, 1, 1000, False, False, None)
    truss.SetPos(ch.ChVector3d(0, -1, 0))
    truss.SetFixed(True)
    truss.SetGravitationalAcceleration(ch.ChVector3d(0, g, 0))
    truss.SetName("truss")
    sys.Add(truss)

    
    rotating_frame = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False, None)
    rotating_frame.SetPos(ch.ChVector3d(2, 0, 0))
    rotating_frame.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    rotating_frame.SetGravitationalAcceleration(ch.ChVector3d(0, g, 0))
    rotating_frame.SetName("rotating frame")
    sys.Add(rotating_frame)

    
    link1 = ch.ChLinkMateGeneric()
    link1.Initialize(box, rotating_frame, False, False, False, False)
    link1.SetConstrainedCoords(True, True, True, False, False, False)
    sys.AddLink(link1)

    
    link2 = ch.ChLinkMateGeneric()
    link2.Initialize(sphere, rotating_frame, False, False, False, False)
    link2.SetConstrainedCoords(True, True, True, False, False, False)
    sys.AddLink(link2)

    
    link3 = ch.ChLinkMateGeneric()
    link3.Initialize(rotating_frame, truss, False, False, False, False)
    link3.SetConstrainedCoords(True, True, True, False, False, False)
    sys.AddLink(link3)

    
    link4 = ch.ChLinkMateGeneric()
    link4.Initialize(box, truss, False, False, False, False)
    link4.SetConstrainedCoords(True, True, True, False, False, False)
    sys.AddLink(link4)

    
    link5 = ch.ChLinkMateGeneric()
    link5.Initialize(sphere, truss, False, False, False, False)
    link5.SetConstrainedCoords(True, True, True, False, False, False)
    sys.AddLink(link5)

    
    link6 = ch.ChLinkMateGeneric()
    link6.Initialize(rotating_frame, truss, False, False, False, False)
    link6.SetConstrainedCoords(True, True, True, False, False, False)
    sys.AddLink(link6)

    
    link7 = ch.ChLinkMateGeneric()
    link7.Initialize(box, truss, False, False, False, False)
    link7.SetConstrainedCoords(True
print("error happened with only start ```python")