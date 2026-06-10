import pychrono as chrono
import pychrono.irrlicht as irr


def main():
    
    
    
    
    
    chrono.SetChronoDataPath("C:/Chrono/data/")
    
    
    my_system = chrono.ChSystemNSC()
    my_system.SetGravity(chrono.ChVector3d(0, -9.81, 0))
    
    
    my_system.SetSolverType(chrono.ChSolver.Type_SOR)
    my_system.SetMaxIterations(100)
    my_system.SetTolerance(1e-6)
    
    
    
    
    
    
    ground = chrono.ChBodyEasyBox(10.0, 1.0, 10.0, 1000, 
                                   chrono.ChMaterialSurface.NSC())
    ground.SetPos(chrono.ChVector3d(0, -5.5, 0))
    ground.SetFixed(True)
    ground.SetName("Ground")
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("texture/concrete.png"))
    my_system.AddBody(ground)
    
    
    mass = chrono.ChBodyEasySphere(0.5, 1000, 
                                    chrono.ChMaterialSurface.NSC())
    mass.SetPos(chrono.ChVector3d(0, 0, 0))
    mass.SetName("Mass")
    
    
    mass_sphere_vis = chrono.ChVisualShapeSphere(0.5)
    mass_sphere_vis.SetColor(chrono.ChColor(1.0, 0.2, 0.2, 1.0))
    mass.AddVisualShape(mass_sphere_vis)
    my_system.AddBody(mass)
    
    
    
    
    
    
    spring_damper = chrono.ChLinkTSDA()
    
    
    anchor_ground = chrono.ChVector3d(0, 0, 0)  
    anchor_mass = chrono.ChVector3d(0, 0, 0)    
    
    
    spring_damper.Initialize(ground, mass, True, anchor_ground, anchor_mass)
    
    
    
    
    
    k = 100.0   
    c = 5.0     
    rest_length = 5.0  
    
    spring_damper.SetSpring(k, True, rest_length)
    spring_damper.SetDamping(c)
    
    
    my_system.AddLink(spring_damper)
    
    
    
    
    
    
    spring_color = chrono.ChColor(0.2, 0.6, 1.0, 1.0)  
    spring_vis = chrono.ChVisualShapeLine()
    spring_vis.SetColor(spring_color)
    spring_vis.SetClosed(False)  
    spring_vis.SetWidth(0.05)
    spring_damper.AddVisualShape(spring_vis)
    
    
    
    
    
    
    myapplication = irr.ChIrrApp(
        my_system,
        "Mass-Spring-Damper System - PyChrono Simulation",
        irr.dimension2du(1280, 720),
        irr.QT_NONE,  
        irr.DEVICE_NULL,
        False  
    )
    
    
    device = irr.CreateDevice irr.DEVICE_OPENGL)
    device.run()
    device.SetWindowCaption("Mass-Spring-Damper Simulation")
    device.SetWindowSize(irr.dimension2du(1280, 720))
    
    
    smgr = device.GetSceneManager()
    myapplication.AttachSystemToDevice(device)
    
    
    camera = smgr.AddCameraSceneNode(
        None,  
        irr.vector3df(8, 4, -8),  
        irr.vector3df(0, 0, 0),   
        -1  
    )
    camera.SetFOV(0.5)  
    camera.SetNearValue(0.1)
    camera.SetFarValue(100.0)
    
    
    camera_anim = irr.ChCameraAnimator(
        irr.vector3df(0, 0, 0),  
        15.0,                     
        0.5                       
    )
    smgr.AddCameraSceneNodeAnimator(camera, camera_anim)
    
    
    
    light1 = smgr.AddLight(
        None,
        irr.vector3df(20, 30, 20),  
        irr.video.SColorf(1.0, 1.0, 1.0, 1.0),  
        50.0,  
        0.0    
    )
    light1.SetLightType(irr.ELT_DIRECTIONAL)
    light1.SetDirection(irr.vector3df(-1, -2, -1))
    light1.EnableCastShadow(True)
    
    
    light2 = smgr.AddLight(
        None,
        irr.vector3df(-10, 10, -10),
        irr.video.SColorf(0.7, 0.7, 0.8, 1.0),  
        40.0,
        0.0
    )
    light2.SetLightType(irr.ELT_POINT)
    
    
    smgr.SetAmbientLight(irr.video.SColorf(0.3, 0.3, 0.4, 1.0))
    
    
    myapplication.AddTypicalSky()
    
    
    grid = chrono.ChLineShape()
    grid.GetLinePoints()
    
    
    
    
    
    
    
    timestep = 0.001  
    
    
    mass.SetPos(chrono.ChVector3d(0, 1.0, 0))  
    mass.SetVel(chrono.ChVector3d(0, 0, 0))     
    
    print("=" * 60)
    print("PyChrono Mass-Spring-Damper Simulation")
    print("=" * 60)
    print(f"Spring Stiffness (k): {k} N/m")
    print(f"Damping Coefficient (c): {c} N·s/m")
    print(f"Rest Length: {rest_length} m")
    print(f"Mass: {mass.GetMass()} kg")
    print("=" * 60)
    print("Press ESC to exit the simulation")
    print("=" * 60)
    
    
    myapplication.SetupXVideoSystem()
    
    
    while device.Run():
        
        device.BeginScene(True, True, irr.video.SColor(255, 50, 50, 50))
        
        
        smgr.DrawAll()
        
        
        driver = device.GetVideoDriver()
        
        
        info_text = f"Time: {my_system.GetChTime():.2f} s"
        pos_text = f"Mass Position Y: {mass.GetPos().y:.3f} m"
        vel_text = f"Mass Velocity Y: {mass.GetVel().y:.3f} m/s"
        force_text = f"Spring Force: {spring_damper.GetForce():.2f} N"
        
        driver.Draw2DText(
            irr.vector2di(10, 10), 
            "PyChrono Mass-Spring-Damper Simulation",
            irr.video.SColor(255, 255, 255, 0)
        )
        driver.Draw2DText(irr.vector2di(10, 30), info_text, irr.video.SColor(255, 255, 255, 0))
        driver.Draw2DText(irr.vector2di(10, 50), pos_text, irr.video.SColor(255, 255, 255, 0))
        driver.Draw2DText(irr.vector2di(10, 70), vel_text, irr.video.SColor(255, 255, 255, 0))
        driver.Draw2DText(irr.vector2di(10, 90), force_text, irr.video.SColor(255, 255, 255, 0))
        
        
        myapplication.DoStep(timestep)
        
        
        device.EndScene()
    
    
    device.Drop()
    
    print("\nSimulation completed successfully!")


if __name__ == '__main__':
    main()