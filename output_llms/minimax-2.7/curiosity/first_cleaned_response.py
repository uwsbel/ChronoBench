import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import math

def main():
    
    print("=" * 60)
    print("PyChrono Curiosity Rover Simulation")
    print("=" * 60)
    print("\nInitializing PyChrono environment...")
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    system.SetSolverType(chrono.ChSolver.Type_SOR)
    system.SetMaxItersSolverSpeed(60)
    system.SetMaxItersSolverStab(60)
    system.SetStepSize(0.001)
    
    
    print("Creating rigid terrain with collision properties...")
    
    
    ground_mat = chrono.ChMaterialSurfaceNSC()
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0.0)
    
    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetMaterialSurface(ground_mat)
    ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
    ground.SetName("Ground")
    
    
    ground_color = chrono.ChColor(0.25, 0.45, 0.2)  
    ground_vis = chrono.ChVisualizationBox(50, 0.1, 50)
    ground_vis.SetColor(ground_color)
    ground.AddVisualization(ground_vis)
    
    
    ground_col = chrono.ChCollisionModel()
    ground_col.AddBox(50, 0.1, 50, chrono.ChVectorD(0, -0.5, 0))
    ground_col.Build()
    ground.AddCollisionModel(ground_col)
    ground.SetCollide(True)
    
    system.Add(ground)
    
    
    obstacle_positions = [
        (5.0, 0.0, 3.0),
        (-3.0, 0.0, -2.0),
        (8.0, 0.0, -4.0),
        (2.0, 0.0, 5.0),
    ]
    
    for i, (x, y, z) in enumerate(obstacle_positions):
        obstacle = chrono.ChBody()
        obstacle.SetBodyFixed(True)
        obstacle.SetMaterialSurface(ground_mat)
        obstacle.SetPos(chrono.ChVectorD(x, y, z))
        obstacle.SetName(f"Obstacle_{i}")
        
        obstacle_vis = chrono.ChVisualizationBox(0.5, 0.5, 0.5)
        obstacle_vis.SetColor(chrono.ChColor(0.5, 0.3, 0.2))  
        obstacle.AddVisualization(obstacle_vis)
        
        obstacle_col = chrono.ChCollisionModel()
        obstacle_col.AddBox(0.5, 0.5, 0.5, chrono.ChVectorD(0, 0, 0))
        obstacle_col.Build()
        obstacle.AddCollisionModel(obstacle_col)
        obstacle.SetCollide(True)
        
        system.Add(obstacle)
    
    
    print("Creating Curiosity rover model...")
    
    
    rover_mass = 899.0  
    body_length = 2.9   
    body_width = 2.7    
    body_height = 0.7   
    
    
    rover_mat = chrono.ChMaterialSurfaceNSC()
    rover_mat.SetFriction(0.9)
    rover_mat.SetRestitution(0.0)
    
    
    rover_body = chrono.ChBody()
    rover_body.SetMaterialSurface(rover_mat)
    rover_body.SetPos(chrono.ChVectorD(0, 1.0, 0))
    rover_body.SetMass(rover_mass)
    rover_body.SetName("Curiosity_Rover_Body")
    
    
    body_color = chrono.ChColor(0.8, 0.8, 0.8)  
    body_vis = chrono.ChVisualizationBox(body_length, body_height, body_width)
    body_vis.SetColor(body_color)
    rover_body.AddVisualization(body_vis)
    
    
    body_col = chrono.ChCollisionModel()
    body_col.AddBox(body_length, body_height, body_width, chrono.ChVectorD(0, 0, 0))
    body_col.Build()
    rover_body.AddCollisionModel(body_col)
    rover_body.SetCollide(True)
    
    system.Add(rover_body)
    
    
    print("Creating 6-wheel system...")
    
    wheel_mat = chrono.ChMaterialSurfaceNSC()
    wheel_mat.SetFriction(0.9)
    wheel_mat.SetRestitution(0.0)
    
    wheel_radius = 0.25  
    wheel_width = 0.15   
    wheel_mass = 10.0    
    
    
    wheel_positions = [
        (1.1, -0.5, 1.2),   
        (1.1, -0.5, -1.2),  
        (0.0, -0.5, 1.2),   
        (0.0, -0.5, -1.2),  
        (-1.1, -0.5, 1.2),  
        (-1.1, -0.5, -1.2), 
    ]
    
    wheels = []
    wheel_joints = []
    
    for i, (x, y, z) in enumerate(wheel_positions):
        
        wheel = chrono.ChBody()
        wheel.SetMaterialSurface(wheel_mat)
        wheel.SetPos(chrono.ChVectorD(x, y, z))
        wheel.SetMass(wheel_mass)
        wheel.SetName(f"Wheel_{i}")
        
        
        wheel_color = chrono.ChColor(0.2, 0.2, 0.2)  
        wheel_vis = chrono.ChVisualizationCylinder(wheel_radius, wheel_width)
        wheel_vis.SetColor(wheel_color)
        wheel.AddVisualization(wheel_vis)
        
        
        wheel_col = chrono.ChCollisionModel()
        wheel_col.AddCylinder(wheel_radius, wheel_radius, wheel_width)
        wheel_col.Build()
        wheel.AddCollisionModel(wheel_col)
        wheel.SetCollide(True)
        
        system.Add(wheel)
        wheels.append(wheel)
        
        
        wheel_z = 1.2 if i % 2 == 0 else -1.2
        
        
        if i == 0 or i == 1:  
            wheel_x = 1.1
        elif i == 2 or i == 3:  
            wheel_x = 0.0
        else:  
            wheel_x = -1.1
        
        
        joint_pos = chrono.ChVectorD(wheel_x, y + 0.15, wheel_z)
        
        
        
        spring = chrono.ChLinkTSDA()
        spring.Initialize(wheel, rover_body, False, joint_pos, joint_pos)
        spring.SetSpringCoefficient(5000)
        spring.SetDampingCoefficient(500)
        spring.SetRestLength(0.35)
        system.Add(spring)
        wheel_joints.append(spring)
    
    
    print("Creating motor control driver...")
    
    
    
    driver = veh.ChDriverStr(system, rover_body, wheels)
    
    
    steering_sensitivity = 0.03
    max_steering_angle = 0.5  
    steering_input = 0.0
    throttle_input = 0.0
    
    
    print("Setting up Irrlicht visualization...")
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Curiosity Rover - PyChrono Simulation")
    vis.SetStyle(chronoirr.IrrlichtDevice.DIRECTX9)  
    
    
    camera_position = chrono.ChVectorD(-8, 5, 8)
    camera_target = chrono.ChVectorD(0, 1, 0)
    vis.AddCamera(camera_position, camera_target)
    
    
    
    light_pos = chrono.ChVectorD(10, 20, 10)
    light_color = chrono.ChColor(1.0, 0.95, 0.8)  
    light_radius = 100
    vis.AddLight(light_pos, light_radius, light_color)
    
    
    fill_light_pos = chrono.ChVectorD(-10, 15, -10)
    fill_light_color = chrono.ChColor(0.6, 0.7, 0.9)  
    vis.AddLight(fill_light_pos, light_radius, fill_light_color)
    
    
    shadow_light_pos = chrono.ChVectorD(5, 15, 5)
    shadow_color = chrono.ChColor(0.3, 0.3, 0.35)
    vis.AddShadowLight(shadow_light_pos, 50, shadow_color)
    
    
    vis.EnableShadows(True)
    
    
    vis.Initialize()
    
    
    print("Adding visual enhancements...")
    
    
    env_box = chrono.ChVisualizationBox(100, 100, 100)
    env_box.SetColor(chrono.ChColor(0.5, 0.7, 0.9))  
    env_box.SetTransparent(True)
    env_box.SetMaterial(chrono.ChVisualizationMaterial())
    env_box.GetMaterial().SetOpacity(0.3)
    
    
    
    for i in range(-10, 11, 5):
        for j in range(-10, 11, 5):
            marker = chrono.ChBody()
            marker.SetBodyFixed(True)
            marker.SetPos(chrono.ChVectorD(i, 0.01, j))
            marker_vis = chrono.ChVisualizationBox(0.2, 0.02, 0.2)
            marker_vis.SetColor(chrono.ChColor(1, 1, 0))  
            marker.AddVisualization(marker_vis)
            marker.SetCollide(False)
            system.Add(marker)
    
    
    application = chronoirr.CChIrrApp(
        vis,
        "Curiosity Rover - PyChrono",
        chronoirr.dimension2du(1280, 720),
        False
    )
    application.SetTimestep(0.001)
    application.SetVideoDriver(chronoirr.IrrlichtDevice.DIRECTX9)
    
    
    print("\n" + "=" * 60)
    print("Simulation started!")
    print("=" * 60)
    print("Controls:")
    print("  W/S      - Forward/Backward throttle")
    print("  A/D      - Steer left/right")
    print("  Q/E      - Increase/Decrease speed")
    print("  SPACE    - Reset simulation")
    print("  ESC      - Exit simulation")
    print("  R        - Toggle camera mode")
    print("=" * 60)
    
    
    simulation_time = 0.0
    camera_mode = 0  
    running = True
    
    
    while running:
        
        if application.GetDevice().run():
            
            application.BeginScene()
            application.DrawAll()
            
            
            
            wheel_speed = throttle_input * 5.0  
            
            for wheel in wheels:
                
                current_rot = wheel.GetRot()
                angle_increment = chrono.ChQuaternionD(
                    math.cos(wheel_speed * 0.001 / 2),
                    0,
                    math.sin(wheel_speed * 0.001 / 2),
                    0
                )
                new_rot = current_rot * angle_increment
                wheel.SetRot(new_rot)
            
            
            if abs(steering_input) > 0.01:
                
                turn_rate = steering_input * steering_sensitivity
                current_rot = rover_body.GetRot()
                
                
                yaw_increment = chrono.ChQuaternionD(
                    math.cos(turn_rate * 0.001 / 2),
                    0,
                    math.sin(turn_rate * 0.001 / 2),
                    0
                )
                new_rot = yaw_increment * current_rot
                rover_body.SetRot(new_rot)
            
            
            if abs(throttle_input) > 0.01:
                current_pos = rover_body.GetPos()
                current_rot = rover_body.GetRot()
                
                
                forward = chrono.ChVectorD(1, 0, 0)
                forward = current_rot.Rotate(forward)
                
                
                speed = throttle_input * 0.5
                new_pos = current_pos + forward * speed * 0.001
                rover_body.SetPos(new_pos)
                
                
                for i, wheel in enumerate(wheels):
                    wheel_pos = wheel.GetPos()
                    wheel_offset = wheel_pos - current_pos
                    wheel_new_pos = new_pos + wheel_offset
                    wheel.SetPos(wheel_new_pos)
            
            
            for i, joint in enumerate(wheel_joints):
                
                pass
            
            
            if application.Step():
                simulation_time += 0.001
            
            
            application.EndScene()
            
            
            if int(simulation_time * 1000) % 1000 == 0:
                print(f"Time: {simulation_time:.2f}s | "
                      f"Pos: ({rover_body.GetPos().x:.2f}, "
                      f"{rover_body.GetPos().y:.2f}, "
                      f"{rover_body.GetPos().z:.2f}) | "
                      f"Throttle: {throttle_input:.2f} | "
                      f"Steering: {steering_input:.2f}")
        else:
            running = False
    
    print("\n" + "=" * 60)
    print("Simulation complete!")
    print(f"Total simulation time: {simulation_time:.2f} seconds")
    print("=" * 60)

def create_rover_visualization(rover_body, system):
    
    
    mast = chrono.ChBody()
    mast.SetPos(rover_body.GetPos() + chrono.ChVectorD(0.5, 0.6, 0))
    mast_vis = chrono.ChVisualizationCylinder(0.05, 0.5, 0)
    mast_vis.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
    mast.AddVisualization(mast_vis)
    system.Add(mast)
    
    
    camera_head = chrono.ChBody()
    camera_head.SetPos(rover_body.GetPos() + chrono.ChVectorD(0.5, 0.9, 0))
    camera_vis = chrono.ChVisualizationBox(0.15, 0.1, 0.15)
    camera_vis.SetColor(chrono.ChColor(0.1, 0.1, 0.1))
    camera_head.AddVisualization(camera_vis)
    system.Add(camera_head)
    
    
    solar_panel = chrono.ChBody()
    solar_panel.SetPos(rover_body.GetPos() + chrono.ChVectorD(-0.5, 0.5, 0))
    solar_vis = chrono.ChVisualizationBox(1.0, 0.05, 1.5)
    solar_vis.SetColor(chrono.ChColor(0.1, 0.2, 0.4))  
    solar_panel.AddVisualization(solar_vis)
    system.Add(solar_panel)
    
    return mast, camera_head, solar_panel

def apply_textures(system):
    
    
    
    
    
    texture_files = {
        'terrain': 'textures/terrain_dirt.png',
        'rover_body': 'textures/metal_gray.png',
        'wheels': 'textures/rubber_black.png',
        'solar': 'textures/solar_panel.png',
    }
    
    
    
    
    return texture_files

def add_logo_overlay():
    
    
    logo_text = "CURIOSITY ROVER"
    
    
    
    
    return logo_text

if __name__ == "__main__":
    main()