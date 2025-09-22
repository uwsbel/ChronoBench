import pychrono.core as chrono
    import pychrono.irrlicht as irr
    import pychrono.vehicle as veh
    import math

    
    sys = chrono.ChSystemNSC()

    
    ground = veh.ChGround()
    ground.SetFixed(True)  
    sys.Add(ground)  

    
    truck = veh.ChTruckMAN10t()
    truck.SetContactMethod(sys.GetContactMethod())  
    sys.Add(truck)  

    
    init_pos = chrono.ChVector3d(0, 0.5, 0)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
    truck.Initialize(chrono.ChFramed(init_pos, init_rot))

    
    driver = veh.ChInteractiveDriverIRR(truck.GetVehicle())
    sys.Add(driver)  

    
    driver.SetSteeringDelta(0.0025)
    driver.SetThrottleDelta(0.0025)

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('MAN 10t Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.75, 0.0), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(truck.GetVehicle())  

    
    time_step = 1e-3

    
    time = 0
    while vis.Run():
        
        driver.SetSteering(vis.GetSteering())
        driver.SetThrottle(vis.GetThrottle())
        driver.SetBraking(vis.GetBraking())

        
        truck.GetVehicle().Update()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        time += time_step

        
        time_step.sleep()