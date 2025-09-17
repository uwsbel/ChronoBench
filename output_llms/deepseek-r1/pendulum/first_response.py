import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    # 1. Initialize the system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

    # 2. Create ground body
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    system.Add(ground)

    # 3. Create pendulum body (rod shape)
    pendulum = chrono.ChBodyEasyBox(0.1, 2.0, 0.1,  # Dimensions (x,y,z)
                                    1000,            # Density (kg/m³)
                                    True, True)      # Enable collision and visualization
    pendulum.SetPos(chrono.ChVectorD(0, -1, 0))      # Position COM below joint
    pendulum.SetRot(chrono.QUNIT)                    # No rotation
    system.Add(pendulum)

    # 4. Create revolute joint at top of pendulum
    revolute = chrono.ChLinkLockRevolute()
    revolute.Initialize(ground, 
                        pendulum, 
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),  # Joint position
                                           chrono.Q_from_AngX(chrono.CH_C_PI_2)))  # Rotate for XY motion
    system.AddLink(revolute)

    # 5. Set up Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Simple Pendulum')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 0, 3))
    vis.AddTypicalLights()

    # 6. Simulation loop
    time_step = 0.001
    log_interval = 0.01  # Log every 10ms
    next_log_time = 0

    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)
        
        # Log position and velocity at specified intervals
        t = system.GetChTime()
        if t >= next_log_time:
            pos = pendulum.GetPos()
            vel = pendulum.GetPos_dt()
            print(f"Time: {t:.3f}s | Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) | "
                  f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
            next_log_time += log_interval

if __name__ == "__main__":
    main()