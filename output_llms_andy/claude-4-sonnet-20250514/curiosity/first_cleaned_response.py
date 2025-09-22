import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import numpy as np

def main():
    
    chrono.SetChronoDataPath('/path/to/chrono/data/')  
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
    
    
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)
    
    
    ground_mat = chrono.ChMaterialSurfaceNSC()
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0.1)
    ground_mat.SetRollingFriction(0.1)
    ground_mat.SetSpinningFriction(0.05)
    
    
    ground = chrono.ChBodyEasyBox(50, 2, 50, 1000, True, True, ground_mat)
    ground.SetPos(chrono.ChVectorD(0, -1, 0))
    ground.SetBodyFixed(True)
    ground.SetName("ground")
    system.Add(ground)
    
    
    rock_mat = chrono.ChMaterialSurfaceNSC()
    rock_mat.SetFriction(0.9)
    rock_mat.SetRestitution(0.3)
    
    
    rock_positions = [
        chrono.ChVectorD(5, 0.5, 3),
        chrono.ChVectorD(-3, 0.8, 8),
        chrono.ChVectorD(8, 0.6, -5),
        chrono.ChVectorD(-7, 0.4, -2),
        chrono.ChVectorD(12, 1.0, 10)
    ]
    
    rocks = []
    for i, pos in enumerate(rock_positions):
        rock_size = np.random.uniform(0.3, 1.2)
        rock = chrono.ChBodyEasyEllipsoid(
            chrono.ChVectorD(rock_size, rock_size*0.8, rock_size*0.9),
            2700, True, True, rock_mat
        )
        rock.SetPos(pos)
        rock.SetBodyFixed(True)
        rock.SetName(f"rock_{i}")
        system.Add(rock)
        rocks.append(rock)
    
    
    rover = robot.Curiosity(system)
    
    
    rover_pos = chrono.ChVectorD(0, 0.5, 0)
    rover_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    rover.Initialize(chrono.ChFrameD(rover_pos, rover_rot))
    
    
    driver = robot.CuriosityDCMotorControl()
    rover.SetDriver(driver)
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Curiosity Rover Terrain Navigation")
    vis.Initialize()
    
    
    vis.AddCamera(chrono.ChVectorD(-8, 4, -8), chrono.ChVectorD(0, 0, 0))
    vis.GetActiveCamera().SetAngle(chrono.CH_C_PI / 3)
    
    
    vis.EnableShadows()
    vis.SetAmbientLight(chrono.ChColor(0.4, 0.4, 0.5))
    
    
    vis.AddLight(chrono.ChVectorD(30, 100, 30), 290, chrono.ChColor(1, 1, 0.9))
    
    
    vis.AddPointLight(chrono.ChVectorD(0, 10, 0), 50, chrono.ChColor(0.8, 0.8, 1))
    
    
    ground_texture = vis.GetVideoDriver().getTexture("textures/ground_mars.jpg")
    if ground_texture:
        ground.GetVisualShape(0).SetTexture(ground_texture)
    
    
    ground.GetVisualShape(0).SetColor(chrono.ChColor(0.7, 0.4, 0.2))  
    
    
    for i, rock in enumerate(rocks):
        color_variation = 0.3 + 0.4 * np.random.random()
        rock.GetVisualShape(0).SetColor(chrono.ChColor(
            0.4 + color_variation * 0.3,
            0.3 + color_variation * 0.2,
            0.2 + color_variation * 0.1
        ))
    
    
    try:
        vis.AddLogo("logo_pychrono_alpha.png")
    except:
        print("Logo file not found, continuing without logo")
    
    
    try:
        vis.SetSkyBox("skybox/")
    except:
        vis.SetSkyBox()  
    
    
    step_size = 0.005
    simulation_time = 0
    max_simulation_time = 60  
    
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    
    steering_angle = 0
    motor_speed = 0
    
    print("Simulation started. Use the following controls:")
    print("- Hold LEFT/RIGHT arrow keys to steer")
    print("- Hold UP/DOWN arrow keys for forward/backward motion")
    print("- ESC to exit")
    
    
    while vis.Run() and simulation_time < max_simulation_time:
        
        receiver = vis.GetDevice().getCursorControl()
        
        
        steering_input = 0
        throttle_input = 0
        
        
        if vis.GetDevice().isKeyPressed(chronoirr.KEY_LEFT):
            steering_input = -1
        elif vis.GetDevice().isKeyPressed(chronoirr.KEY_RIGHT):
            steering_input = 1
            
        if vis.GetDevice().isKeyPressed(chronoirr.KEY_UP):
            throttle_input = 1
        elif vis.GetDevice().isKeyPressed(chronoirr.KEY_DOWN):
            throttle_input = -1
        
        
        steering_angle += (steering_input - steering_angle) * 0.1
        motor_speed += (throttle_input - motor_speed) * 0.05
        
        
        driver_inputs = robot.CuriosityDriverInputs()
        driver_inputs.m_steering = steering_angle
        driver_inputs.m_throttle = motor_speed
        
        
        rover.Update(simulation_time, driver_inputs)
        
        
        rover_pos = rover.GetChassis().GetPos()
        camera_pos = rover_pos + chrono.ChVectorD(-10, 6, -10)
        look_at = rover_pos + chrono.ChVectorD(0, 0, 5)
        vis.GetActiveCamera().setTarget(
            chronoirr.vector3df(look_at.x, look_at.y, look_at.z)
        )
        vis.GetActiveCamera().setPosition(
            chronoirr.vector3df(camera_pos.x, camera_pos.y, camera_pos.z)
        )
        
        
        vis.BeginScene()
        vis.Render()
        
        
        vis.GetGUIEnvironment().getBuiltInFont().draw(
            f"Time: {simulation_time:.1f}s",
            chronoirr.recti(10, 10, 200, 30),
            chronoirr.SColor(255, 255, 255, 255)
        )
        
        vis.GetGUIEnvironment().getBuiltInFont().draw(
            f"Rover Speed: {rover.GetChassisVel().Length():.2f} m/s",
            chronoirr.recti(10, 35, 300, 55),
            chronoirr.SColor(255, 255, 255, 255)
        )
        
        vis.GetGUIEnvironment().getBuiltInFont().draw(
            f"Position: X={rover_pos.x:.1f}, Z={rover_pos.z:.1f}",
            chronoirr.recti(10, 60, 400, 80),
            chronoirr.SColor(255, 255, 255, 255)
        )
        
        vis.EndScene()
        
        
        system.DoStepDynamics(step_size)
        simulation_time += step_size
        
        
        realtime_timer.Spin(step_size)
    
    print("Simulation completed successfully!")

if __name__ == "__main__":
    main()