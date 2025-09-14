import os
import pychrono as chrono
import pychrono.irrlicht as irrlicht
import pychrono.robot as robot

def main():
    # 1. Initialize PyChrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    # 2. Create terrain
    terrain = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
    terrain.SetPos(chrono.ChVectorD(0, -0.5, 0))
    terrain.SetBodyFixed(True)
    terrain.GetVisualShape(0).SetTexture(chrono.GetChronoDataPath() + "textures/concrete.jpg")
    system.Add(terrain)

    # 3. Create Curiosity rover
    rover = robot.Curiosity()
    rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
    rover.SetDriverInputs(0, 0, 0)  # Initial inputs
    system.Add(rover.GetChassis())

    # 4. Set up visualization
    application = irrlicht.ChIrrApp(system, "Curiosity Rover Simulation", irrlicht.dimension2du(1280, 720))
    application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
    application.AddTypicalSky()
    application.AddTypicalCamera(irrlicht.vector3df(3, 2, 3), irrlicht.vector3df(0, 0, 0))
    application.AddLightWithShadow(irrlicht.vector3df(10, 10, 10), irrlicht.vector3df(0, 0, 0), 50, 5, 50, 55, 512)
    application.EnableShadows()

    # 5. Motor control parameters
    steering = 0.0
    speed = 0.5
    max_steering = 0.3

    # 6. Simulation loop
    application.AssetBindAll()
    application.AssetUpdateAll()
    time_step = 0.001

    while application.GetDevice().run():
        application.BeginScene()
        application.DrawAll()
        
        # Handle input
        driver_inputs = rover.GetDriverInputs()
        if application.GetDevice().getEventReceiver().IsKeyDown(irrlicht.KEY_UPARROW):
            speed = min(speed + 0.01, 2.0)
        if application.GetDevice().getEventReceiver().IsKeyDown(irrlicht.KEY_DOWNARROW):
            speed = max(speed - 0.01, 0.0)
        if application.GetDevice().getEventReceiver().IsKeyDown(irrlicht.KEY_LEFTARROW):
            steering = max(steering - 0.01, -max_steering)
        if application.GetDevice().getEventReceiver().IsKeyDown(irrlicht.KEY_RIGHTARROW):
            steering = min(steering + 0.01, max_steering)

        # Update rover controls
        rover.SetDriverInputs(steering, speed, speed)
        
        # Update camera to follow rover
        rover_pos = rover.GetChassis().GetPos()
        application.GetActiveCamera().setTarget(irrlicht.vector3df(rover_pos.x, rover_pos.y, rover_pos.z))
        
        # Simulation step
        application.DoStepDynamic(time_step)
        application.EndScene()

if __name__ == "__main__":
    main()