import pychrono.core as chrono
    import pychrono.irrlicht as irr
    import pychrono.vehicle as veh
    import math

    
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  

    
    contact_material = chrono.ChContactMaterialNSC()
    contact_material.SetFriction(0.9)
    contact_material.SetRestitution(0.01)

    
    bus = veh.CityBus(sys, contact_material)
    bus.SetContactMethod(chrono.ChContactMethod_NSC)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    bus.SetTireType(veh.TireModelType_TMEASY)
    bus.SetTireStepSize(1e-3)
    bus.SetInitFwdVel(0.0)
    bus.SetMaxMotorVoltageRatio(0.16)
    bus.SetStallTorque(0.3)
    bus.SetTireRollingResistance(0.06)

    
    bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
    bus.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    driver = veh.ChInteractiveDriverIRR(bus.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    terrain = veh.RigidTerrain(sys)
    terrain.SetSoilParameters(2e6,   
                                0,     
                                1.1,   
                                0,     
                                30,    
                                0.01,  
                                2e8,   
                                3e4    
                               )
    terrain.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('CityBus Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(bus.GetVehicle())

    
    time_step = 1e-3

    
    time = 0
    while vis.Run():
        time += time_step

        
        driver.SetSteering(0.5)
        driver.SetThrottle(0.2)
        driver.SetBraking(0.0)

        
        bus_state = bus.GetVehicle().GetState()
        print('Time:', time)
        print('  Location:', bus_state.pos)
        print('  Linear Velocity:', bus_state.vel)
        print('  Angular Velocity:', bus_state.ang_vel)

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver.Synchronize(time_step)
        terrain.Synchronize(time_step)
        bus.Synchronize(time_step)
        vis.Synchronize(time_step)