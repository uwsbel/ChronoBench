import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.irrlicht as chronoirr

def main():
    # 1. Initialize system with gravity
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # 2. Create rigid terrain
    ground = chrono.ChBodyEasyBox(20, 0.1, 20, 1000)
    ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
    ground.SetBodyFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(ground)

    # 3. Initialize TurtleBot at specific position/orientation
    bot = turtlebot.TurtleBot(system)
    bot.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, -0.2, 0), chrono.Q_from_AngY(chrono.CH_C_PI/2)))
    
    # 4. Set up Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("TurtleBot Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 1.5, 3), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), 10, 2, 10, 40, 512)

    # 5. Simulation parameters
    time_step = 0.01
    current_time = 0
    simulation_duration = 5.0  # seconds

    # 6. Main simulation loop
    while vis.Run() and current_time < simulation_duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Apply motor controls based on simulation time
        if current_time < 1.0:
            # Forward motion (first second)
            bot.SetMotorSpeeds(1.0, 1.0)  # Both wheels same speed
        elif current_time < 2.0:
            # Left turn (second second)
            bot.SetMotorSpeeds(0.5, 1.5)  # Right wheel faster
        elif current_time < 3.0:
            # Right turn (third second)
            bot.SetMotorSpeeds(1.5, 0.5)  # Left wheel faster
        else:
            # Forward again (remaining time)
            bot.SetMotorSpeeds(1.0, 1.0)

        # Advance simulation
        bot.Update()
        system.DoStepDynamics(time_step)
        current_time += time_step

if __name__ == "__main__":
    main()