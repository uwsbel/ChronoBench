import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.robot as robot

def main():
    
    
    
    
    
    chrono.SetChronoDataPath("../../../data/")
    
    
    my_system = chrono.ChSystemNSC()
    
    
    my_system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))
    
    
    timestep = 0.001
    
    
    
    
    
    
    ground_mat = chrono.ChMaterialSurfaceNSC()
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0.0)
    
    
    ground_size = chrono.ChVectorD(20, 1, 20)  
    ground = chrono.ChBodyEasyBox(
        ground_size.x, ground_size.y, ground_size.z,
        1000,  
        ground_mat,
        True,   
        False   
    )
    ground.SetPos(chrono.ChVectorD(0, -0.5, 0))  
    ground.SetBodyFixed(True)  
    ground.SetName("Ground")
    
    my_system.Add(ground)
    
    
    
    
    
    
    robot_start_pos = chrono.ChVectorD(0, 0.5, 0)
    robot_start_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
    
    
    
    turtlebot = robot.Turtlebot()
    turtlebot.SetName("Turtlebot")
    
    
    robot_frame = chrono.ChFrameD(robot_start_pos, robot_start_rot)
    turtlebot.Initialize(robot_frame, my_system)
    
    
    
    motor_left = turtlebot.GetMotorDriver(robot.TurtlebotMotor.MOTOR_LEFT)
    motor_right = turtlebot.GetMotorDriver(robot.TurtlebotMotor.MOTOR_RIGHT)
    
    
    motor_left.SetVelFunction(chrono.ChFunction_Const(0))
    motor_right.SetVelFunction(chrono.ChFunction_Const(0))
    
    
    
    
    
    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(my_system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Turtlebot Simulation")
    vis.SetStyle(irr.IrrlichtVisualizationSettings.ERDS_style)
    
    
    if not vis.Initialize():
        print("Error: Could not initialize Irrlicht visualization!")
        return
    
    
    
    camera_distance = 5.0
    camera_height = 3.0
    vis.AddCamera(
        chrono.ChVectorD(0, camera_height, -camera_distance),  
        chrono.ChVectorD(0, 0, 0)  
    )
    
    
    
    vis.AddLight(
        chrono.ChVectorD(10, 20, -10),  
        1.0,                            
        chrono.ChColor(1, 1, 1),        
        chrono.ChVectorD(-0.5, -1, 0.5) 
    )
    
    
    vis.AddLight(
        chrono.ChVectorD(-10, 10, -10),
        0.5,
        chrono.ChColor(0.8, 0.8, 1.0),  
        chrono.ChVectorD(0.5, -1, -0.5)
    )
    
    
    vis.SetAmbientLight(chrono.ChColor(0.3, 0.3, 0.3))
    
    
    
    
    
    
    sim_time = 0.0
    render_time = 0.0
    render_step = 1.0 / 30.0  
    
    
    forward_speed = 3.0      
    turn_speed = 2.0         
    turn_duration = 2.0      
    straight_duration = 3.0  
    
    
    control_sequence = [
        (0.0, forward_speed, forward_speed),  
        (straight_duration, forward_speed, forward_speed),  
        (straight_duration + turn_duration, -turn_speed, turn_speed),  
        (straight_duration * 2 + turn_duration, forward_speed, forward_speed),  
        (straight_duration * 2 + turn_duration * 2, turn_speed, -turn_speed),  
        (straight_duration * 3 + turn_duration * 2, forward_speed, forward_speed),  
    ]
    
    print("=" * 60)
    print("PyChrono Turtlebot Simulation")
    print("=" * 60)
    print("Controls:")
    print("  LEFT ARROW  : Turn left (increase left motor)")
    print("  RIGHT ARROW : Turn right (increase right motor)")
    print("  UP ARROW    : Drive forward")
    print("  DOWN ARROW  : Stop/Reverse")
    print("  SPACE       : Reset simulation")
    print("  ESC         : Exit")
    print("=" * 60)
    print(f"Simulation started at t = {sim_time:.3f}s")
    print("-" * 60)
    
    
    while vis.Run():
        
        key_event = vis.GetKeyEvent()
        if key_event.key_state == irr.EKEY_STATE.EKEY_DOWN:
            if key_event.key == irr.EKEY_CODE.KEY_ESCAPE:
                print("\nSimulation terminated by user.")
                break
            elif key_event.key == irr.EKEY_CODE.KEY_SPACE:
                
                sim_time = 0.0
                turtlebot.Initialize(robot_frame, my_system)
                motor_left.SetVelFunction(chrono.ChFunction_Const(0))
                motor_right.SetVelFunction(chrono.ChFunction_Const(0))
                print("Simulation reset!")
            elif key_event.key == irr.EKEY_CODE.KEY_LEFT:
                motor_left.SetVelFunction(chrono.ChFunction_Const(-turn_speed))
                motor_right.SetVelFunction(chrono.ChFunction_Const(turn_speed))
            elif key_event.key == irr.EKEY_CODE.KEY_RIGHT:
                motor_left.SetVelFunction(chrono.ChFunction_Const(turn_speed))
                motor_right.SetVelFunction(chrono.ChFunction_Const(-turn_speed))
            elif key_event.key == irr.EKEY_CODE.KEY_UP:
                motor_left.SetVelFunction(chrono.ChFunction_Const(forward_speed))
                motor_right.SetVelFunction(chrono.ChFunction_Const(forward_speed))
            elif key_event.key == irr.EKEY_CODE.KEY_DOWN:
                motor_left.SetVelFunction(chrono.ChFunction_Const(0))
                motor_right.SetVelFunction(chrono.ChFunction_Const(0))
        
        
        
        for i in range(len(control_sequence) - 1):
            start_time, left_speed, right_speed = control_sequence[i]
            end_time, _, _ = control_sequence[i + 1]
            if start_time <= sim_time < end_time:
                motor_left.SetVelFunction(chrono.ChFunction_Const(left_speed))
                motor_right.SetVelFunction(chrono.ChFunction_Const(right_speed))
                break
        
        
        my_system.DoStepDynamics(timestep)
        sim_time += timestep
        
        
        render_time += timestep
        if render_time >= render_step:
            vis.BeginScene()
            vis.Render()
            
            
            vis.AddLogo(chrono.ChVectorD(0.02, 0.98, 0), 
                       chrono.ChColor(1, 1, 1), 
                       chrono.CameraPosition(0, 0, 0))
            
            
            vis.DrawAll()
            
            
            vis.SetWindowTitle(f"PyChrono Turtlebot - Time: {sim_time:.2f}s")
            
            vis.EndScene()
            render_time = 0.0
        
        
        robot_pos = turtlebot.GetChassis().GetPos()
        
        
        if int(sim_time * 10) % 20 == 0 and (sim_time - timestep) % 2 < timestep:
            print(f"t = {sim_time:6.2f}s | Robot at: ({robot_pos.x:6.2f}, {robot_pos.y:6.2f}, {robot_pos.z:6.2f})")
    
    
    vis.Close()
    
    print("-" * 60)
    print(f"Simulation ended at t = {sim_time:.2f}s")
    print(f"Final robot position: ({robot_pos.x:.2f}, {robot_pos.y:.2f}, {robot_pos.z:.2f})")
    print("=" * 60)

if __name__ == "__main__":
    main()