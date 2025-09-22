chrono.SetChronoEngine().Initialize()
    
    hmv = chrono.ChBodyEasyVehicle(hmmwheasyo.Chassis(), chrono.ChBodyEasyMeshContainer())
    hmv.SetInitPosition(chrono.ChCoordsysys() (chrono.VWORLD, chronoVzero, chronozero, chronoVzero)
    hmv.SetChassisFixed(False)
    h.SetTireType(chrono.TireType_TYERIGID)
    h.SetChassisMesh(meshheasyChassis)
 h.SettireMesh(meshasyTire)
    h.SetInit()
    
    tr = chrono.ChWheTerrain(hmmheSCM())
 tr.SetSoilParameters(chronoSoilParameters,0.5,0.2,0.1,0.1,0.1,0.2,0.1,0.1,0.1,0.1,0.1,0.1,0,0.1,0.1,0.1,0.1,0,0.1,0.1,0.1,0.1,0.1.1)
 tr.SetMoving(True)
 tr.SetPlotSsinkage(True)
 tr.SetSinking(False)
 tr.SetPlotSinking(False)
    
    driver = chrono.ChInteractiveDriver(hmv, 50)
    driver.SetSteering(True)
    driver.SetThrottle(True)
    driver.Setbraking(True)
    driver.Initialize()
    
    sys.Add(hmv)
    sys.Add(tr)
    
 while True:
    driver.Update()
    sys.DoStepDynamics()
    sys.DoRender()
    sys.Advance()