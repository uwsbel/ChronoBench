import pychrono.core as chrono
    import pychrono.irrlicht as irr
    import pychrono.robot as robot
    import pychrono.vehicle as veh
    import math

    
    sys = chrono.ChSystemNSC()

    
    contact_mat = chrono.ChContactMaterialNSC()
    contact_mat.SetFriction(0.9)
    contact_mat.SetRestitution(0.01)

    
    ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, contact_mat)
    ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  
    ground.SetFixed(True)  
    ground.GetVisualShape(0).SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    ground.SetName("ground")
    sys.Add(ground)  

    
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain = veh.RigidTerrain(sys, ground)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)), 20, 20)
    patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Build()  

    
    rover = robot.Curiosity(sys, contact_mat)
    rover.SetPos(chrono.ChVector3d(0, 0.2, 0))  
    rover.SetDriver(robot.ChWheeledRobotDCMotorControl())
    rover.GetDriver().SetSteeringDelta(0.0)  
    sys.Add(rover)  

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)  
    vis.SetWindowSize(1280, 1024)  
    vis.SetWindowTitle('Curiosity rover - Rigid terrain')  
    vis.Initialize()  
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
    vis.AddSkyBox()  
    vis.AddCamera(chrono.ChVector3d(0, 2.5, 0.2), chrono.ChVector3d(0, 0, 0.2))  
    vis.AddTypicalLights()  
    vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)  

    
    time_step = 1e-3

    
    time = 0
    while vis.Run():
        
        if time < 1:
            rover.GetDriver().SetSteeringDelta(0.2)
        elif time < 3:
            rover.GetDriver().SetSteeringDelta(0.0)
        else:
            rover.GetDriver().SetSteeringDelta(-0.2)
        
        
        rover.Update()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        sys.DoStepDynamics(time_step)

        
        time += time_step