#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PyChrono Simulation: Single Mass-Spring-Damper System
This script demonstrates the use of ChLinkTSDA to connect a mass to ground
with Irrlicht visualization including proper lighting and camera setup.
"""

import pychrono as chrono
import pychrono.irrlicht as irr


def main():
    # ==========================================================================
    # 1. INITIALIZE PYCHRONO ENVIRONMENT
    # ==========================================================================
    
    # Set the data path for Chrono assets (textures, etc.)
    chrono.SetChronoDataPath("C:/Chrono/data/")
    
    # Create the physical system
    my_system = chrono.ChSystemNSC()
    my_system.SetGravity(chrono.ChVector3d(0, -9.81, 0))
    
    # Set solver parameters for better convergence
    my_system.SetSolverType(chrono.ChSolver.Type_SOR)
    my_system.SetMaxIterations(100)
    my_system.SetTolerance(1e-6)
    
    # ==========================================================================
    # 2. CREATE PHYSICAL OBJECTS
    # ==========================================================================
    
    # --- Ground Body ---
    ground = chrono.ChBodyEasyBox(10.0, 1.0, 10.0, 1000, 
                                   chrono.ChMaterialSurface.NSC())
    ground.SetPos(chrono.ChVector3d(0, -5.5, 0))
    ground.SetFixed(True)
    ground.SetName("Ground")
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("texture/concrete.png"))
    my_system.AddBody(ground)
    
    # --- Mass Body ---
    mass = chrono.ChBodyEasySphere(0.5, 1000, 
                                    chrono.ChMaterialSurface.NSC())
    mass.SetPos(chrono.ChVector3d(0, 0, 0))
    mass.SetName("Mass")
    
    # Add visual shape for the mass (red color)
    mass_sphere_vis = chrono.ChVisualShapeSphere(0.5)
    mass_sphere_vis.SetColor(chrono.ChColor(1.0, 0.2, 0.2, 1.0))
    mass.AddVisualShape(mass_sphere_vis)
    my_system.AddBody(mass)
    
    # ==========================================================================
    # 3. CREATE SPRING-DAMPER CONNECTION (ChLinkTSDA)
    # ==========================================================================
    
    # Create the translational spring-damper-actuator link
    spring_damper = chrono.ChLinkTSDA()
    
    # Define anchor points
    anchor_ground = chrono.ChVector3d(0, 0, 0)  # Point on ground
    anchor_mass = chrono.ChVector3d(0, 0, 0)    # Point on mass
    
    # Initialize the link with body references and anchor points
    spring_damper.Initialize(ground, mass, True, anchor_ground, anchor_mass)
    
    # Set spring-damper parameters using default linear model
    # Spring constant (k): stiffness
    # Damping coefficient (c): damping
    # Rest length: natural length of the spring
    k = 100.0   # Spring stiffness [N/m]
    c = 5.0     # Damping coefficient [N·s/m]
    rest_length = 5.0  # Rest length [m]
    
    spring_damper.SetSpring(k, True, rest_length)
    spring_damper.SetDamping(c)
    
    # Add the spring-damper link to the system
    my_system.AddLink(spring_damper)
    
    # ==========================================================================
    # 4. CREATE SPRING VISUAL REPRESENTATION
    # ==========================================================================
    
    # Create a visual line shape to represent the spring
    spring_color = chrono.ChColor(0.2, 0.6, 1.0, 1.0)  # Blue spring
    spring_vis = chrono.ChVisualShapeLine()
    spring_vis.SetColor(spring_color)
    spring_vis.SetClosed(False)  # Open spiral for spring representation
    spring_vis.SetWidth(0.05)
    spring_damper.AddVisualShape(spring_vis)
    
    # ==========================================================================
    # 5. SETUP IRRLICHT VISUALIZATION
    # ==========================================================================
    
    # Create the Irrlicht application
    myapplication = irr.ChIrrApp(
        my_system,
        "Mass-Spring-Damper System - PyChrono Simulation",
        irr.dimension2du(1280, 720),
        irr.QT_NONE,  # Use OpenGL rendering
        irr.DEVICE_NULL,
        False  # Don't create an own window
    )
    
    # Setup the Irrlicht device
    device = irr.CreateDevice irr.DEVICE_OPENGL)
    device.run()
    device.SetWindowCaption("Mass-Spring-Damper Simulation")
    device.SetWindowSize(irr.dimension2du(1280, 720))
    
    # Create and attach the scene manager
    smgr = device.GetSceneManager()
    myapplication.AttachSystemToDevice(device)
    
    # --- Camera Settings ---
    camera = smgr.AddCameraSceneNode(
        None,  # Parent node
        irr.vector3df(8, 4, -8),  # Camera position
        irr.vector3df(0, 0, 0),   # Look-at target
        -1  # Camera ID
    )
    camera.SetFOV(0.5)  # Field of view in radians
    camera.SetNearValue(0.1)
    camera.SetFarValue(100.0)
    
    # Add camera animator to orbit around the scene
    camera_anim = irr.ChCameraAnimator(
        irr.vector3df(0, 0, 0),  # Target point
        15.0,                     # Radius
        0.5                       # Movement speed
    )
    smgr.AddCameraSceneNodeAnimator(camera, camera_anim)
    
    # --- Lighting Settings ---
    # Main directional light
    light1 = smgr.AddLight(
        None,
        irr.vector3df(20, 30, 20),  # Position
        irr.video.SColorf(1.0, 1.0, 1.0, 1.0),  # White light
        50.0,  # Range
        0.0    # Attenuation
    )
    light1.SetLightType(irr.ELT_DIRECTIONAL)
    light1.SetDirection(irr.vector3df(-1, -2, -1))
    light1.EnableCastShadow(True)
    
    # Secondary fill light
    light2 = smgr.AddLight(
        None,
        irr.vector3df(-10, 10, -10),
        irr.video.SColorf(0.7, 0.7, 0.8, 1.0),  # Slightly blue fill
        40.0,
        0.0
    )
    light2.SetLightType(irr.ELT_POINT)
    
    # Ambient light for overall scene illumination
    smgr.SetAmbientLight(irr.video.SColorf(0.3, 0.3, 0.4, 1.0))
    
    # --- Add Skybox ---
    myapplication.AddTypicalSky()
    
    # --- Add Grid ---
    grid = chrono.ChLineShape()
    grid.GetLinePoints()
    # Note: Grid visualization is handled by Chrono's built-in visualization
    
    # ==========================================================================
    # 6. SIMULATION LOOP
    # ==========================================================================
    
    # Set simulation timestep
    timestep = 0.001  # 1 ms timestep for smooth simulation
    
    # Apply initial perturbation to start the oscillation
    mass.SetPos(chrono.ChVector3d(0, 1.0, 0))  # Displace mass 1m down
    mass.SetVel(chrono.ChVector3d(0, 0, 0))     # Start from rest
    
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
    
    # Initialize the application
    myapplication.SetupXVideoSystem()
    
    # Run the visualization loop
    while device.Run():
        # Begin the scene
        device.BeginScene(True, True, irr.video.SColor(255, 50, 50, 50))
        
        # Draw the scene
        smgr.DrawAll()
        
        # Draw GUI info
        driver = device.GetVideoDriver()
        
        # Draw info text
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
        
        # Advance the simulation
        myapplication.DoStep(timestep)
        
        # End the scene
        device.EndScene()
    
    # Cleanup
    device.Drop()
    
    print("\nSimulation completed successfully!")


if __name__ == '__main__':
    main()