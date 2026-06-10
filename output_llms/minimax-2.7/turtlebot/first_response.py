#!/usr/bin/env python3

"""
PyChrono Turtlebot Simulation
-----------------------------
This script creates a simulation of a Turtlebot robot on rigid terrain
with motor control for left and right turns at specified times.
"""

import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.robot as robot

def main():
    # =========================================================================
    # 1. INITIALIZE THE PYCHRONO ENVIRONMENT
    # =========================================================================
    
    # Set the path to Chrono data directory for assets
    chrono.SetChronoDataPath("../../../data/")
    
    # Create a Chrono physical system
    my_system = chrono.ChSystemNSC()
    
    # Set gravity (negative Y direction)
    my_system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))
    
    # Set simulation timestep
    timestep = 0.001
    
    # =========================================================================
    # 2. CREATE THE GROUND BODY (RIGID TERRAIN)
    # =========================================================================
    
    # Create material for ground contact
    ground_mat = chrono.ChMaterialSurfaceNSC()
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0.0)
    
    # Create ground body (large flat surface)
    ground_size = chrono.ChVectorD(20, 1, 20)  # Width, Height, Length
    ground = chrono.ChBodyEasyBox(
        ground_size.x, ground_size.y, ground_size.z,
        1000,  # Density (kg/m^3)
        ground_mat,
        True,   # Visualize
        False   # Collision
    )
    ground.SetPos(chrono.ChVectorD(0, -0.5, 0))  # Position (center at y=-0.5)
    ground.SetBodyFixed(True)  # Ground is fixed (immovable)
    ground.SetName("Ground")
    
    my_system.Add(ground)
    
    # =========================================================================
    # 3. INITIALIZE THE TURTLEBOT ROBOT
    # =========================================================================
    
    # Robot configuration
    robot_start_pos = chrono.ChVectorD(0, 0.5, 0)
    robot_start_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # No rotation (facing +Z)
    
    # Create the Turtlebot using the PyChrono robot module
    # The Turtlebot factory creates a complete robot with chassis, wheels, motors
    turtlebot = robot.Turtlebot()
    turtlebot.SetName("Turtlebot")
    
    # Initialize the robot at specified position and orientation
    robot_frame = chrono.ChFrameD(robot_start_pos, robot_start_rot)
    turtlebot.Initialize(robot_frame, my_system)
    
    # Get motor controllers for velocity control
    # Turtlebot has left and right wheel motors
    motor_left = turtlebot.GetMotorDriver(robot.TurtlebotMotor.MOTOR_LEFT)
    motor_right = turtlebot.GetMotorDriver(robot.TurtlebotMotor.MOTOR_RIGHT)
    
    # Set initial motor parameters
    motor_left.SetVelFunction(chrono.ChFunction_Const(0))
    motor_right.SetVelFunction(chrono.ChFunction_Const(0))
    
    # =========================================================================
    # 4. CREATE IRRLICHT VISUALIZATION
    # =========================================================================
    
    # Create Irrlicht visualization system
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(my_system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Turtlebot Simulation")
    vis.SetStyle(irr.IrrlichtVisualizationSettings.ERDS_style)
    
    # Initialize the visualization
    if not vis.Initialize():
        print("Error: Could not initialize Irrlicht visualization!")
        return
    
    # Set up camera
    # Position camera behind and above the robot
    camera_distance = 5.0
    camera_height = 3.0
    vis.AddCamera(
        chrono.ChVectorD(0, camera_height, -camera_distance),  # Camera position
        chrono.ChVectorD(0, 0, 0)  # Look-at target
    )
    
    # Configure lighting
    # Main directional light
    vis.AddLight(
        chrono.ChVectorD(10, 20, -10),  # Position
        1.0,                            # Intensity
        chrono.ChColor(1, 1, 1),        # Color (white)
        chrono.ChVectorD(-0.5, -1, 0.5) # Direction
    )
    
    # Fill light
    vis.AddLight(
        chrono.ChVectorD(-10, 10, -10),
        0.5,
        chrono.ChColor(0.8, 0.8, 1.0),  # Slightly blue
        chrono.ChVectorD(0.5, -1, -0.5)
    )
    
    # Ambient light
    vis.SetAmbientLight(chrono.ChColor(0.3, 0.3, 0.3))
    
    # =========================================================================
    # 5. SIMULATION LOOP WITH MOTOR CONTROL
    # =========================================================================
    
    # Simulation timing
    sim_time = 0.0
    render_time = 0.0
    render_step = 1.0 / 30.0  # ~30 FPS rendering
    
    # Motor control parameters
    forward_speed = 3.0      # rad/s for wheel angular velocity
    turn_speed = 2.0         # rad/s during turns
    turn_duration = 2.0      # seconds for each turn
    straight_duration = 3.0  # seconds to drive straight between turns
    
    # Define the control sequence (time, left_speed, right_speed)
    control_sequence = [
        (0.0, forward_speed, forward_speed),  # Drive forward
        (straight_duration, forward_speed, forward_speed),  # Continue forward
        (straight_duration + turn_duration, -turn_speed, turn_speed),  # Left turn
        (straight_duration * 2 + turn_duration, forward_speed, forward_speed),  # Resume forward
        (straight_duration * 2 + turn_duration * 2, turn_speed, -turn_speed),  # Right turn
        (straight_duration * 3 + turn_duration * 2, forward_speed, forward_speed),  # Continue forward
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
    
    # Main simulation loop
    while vis.Run():
        # Check for user input events
        key_event = vis.GetKeyEvent()
        if key_event.key_state == irr.EKEY_STATE.EKEY_DOWN:
            if key_event.key == irr.EKEY_CODE.KEY_ESCAPE:
                print("\nSimulation terminated by user.")
                break
            elif key_event.key == irr.EKEY_CODE.KEY_SPACE:
                # Reset the simulation
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
        
        # Automatic motor control based on time (if no key pressed)
        # Check the control sequence
        for i in range(len(control_sequence) - 1):
            start_time, left_speed, right_speed = control_sequence[i]
            end_time, _, _ = control_sequence[i + 1]
            if start_time <= sim_time < end_time:
                motor_left.SetVelFunction(chrono.ChFunction_Const(left_speed))
                motor_right.SetVelFunction(chrono.ChFunction_Const(right_speed))
                break
        
        # Advance the physics simulation
        my_system.DoStepDynamics(timestep)
        sim_time += timestep
        
        # Render at specified frame rate
        render_time += timestep
        if render_time >= render_step:
            vis.BeginScene()
            vis.Render()
            
            # Add text overlay with simulation info
            vis.AddLogo(chrono.ChVectorD(0.02, 0.98, 0), 
                       chrono.ChColor(1, 1, 1), 
                       chrono.CameraPosition(0, 0, 0))
            
            # Draw text showing simulation time
            vis.DrawAll()
            
            # Update window title with time
            vis.SetWindowTitle(f"PyChrono Turtlebot - Time: {sim_time:.2f}s")
            
            vis.EndScene()
            render_time = 0.0
        
        # Get robot position for display
        robot_pos = turtlebot.GetChassis().GetPos()
        
        # Print status periodically (every 2 seconds of simulation time)
        if int(sim_time * 10) % 20 == 0 and (sim_time - timestep) % 2 < timestep:
            print(f"t = {sim_time:6.2f}s | Robot at: ({robot_pos.x:6.2f}, {robot_pos.y:6.2f}, {robot_pos.z:6.2f})")
    
    # Cleanup
    vis.Close()
    
    print("-" * 60)
    print(f"Simulation ended at t = {sim_time:.2f}s")
    print(f"Final robot position: ({robot_pos.x:.2f}, {robot_pos.y:.2f}, {robot_pos.z:.2f})")
    print("=" * 60)

if __name__ == "__main__":
    main()