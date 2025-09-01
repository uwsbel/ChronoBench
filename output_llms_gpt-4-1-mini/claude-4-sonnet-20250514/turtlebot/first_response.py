import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.irrlicht as chronoirr
import math
import time

# ---------------------------------------------------------------------
# Initialize the PyChrono environment and core components
# ---------------------------------------------------------------------

# Create the Chrono system
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Set solver settings for better stability
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)
system.SetMaxPenetrationRecoverySpeed(1.0)

# ---------------------------------------------------------------------
# Create the ground/terrain
# ---------------------------------------------------------------------

# Create ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)

# Create ground shape
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(10, 10, 0.1)
ground.AddVisualShape(ground_shape, chrono.ChFrameD())

# Create ground collision shape
ground_collision = chrono.ChCollisionShapeBox(10, 10, 0.1)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddShape(ground_collision, chrono.ChFrameD())
ground.GetCollisionModel().BuildModel()

# Set ground material properties
ground_material = chrono.ChMaterialSurfaceNSC()
ground_material.SetFriction(0.8)
ground_material.SetRestitution(0.1)
ground.SetMaterialSurface(ground_material)

# Set ground position
ground.SetPos(chrono.ChVectorD(0, 0, -0.1))

# Add ground to system
system.Add(ground)

# ---------------------------------------------------------------------
# Initialize the Turtlebot robot
# ---------------------------------------------------------------------

# Create Turtlebot robot
robot = turtlebot.TurtleBot(system, chrono.ChVectorD(0, 0, 0.1))

# Set initial position and orientation
init_pos = chrono.ChVectorD(0, 0, 0.2)  # Start slightly above ground
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # No initial rotation

robot.Initialize(chrono.ChCoordsysD(init_pos, init_rot))

# Set robot material properties
robot_material = chrono.ChMaterialSurfaceNSC()
robot_material.SetFriction(0.8)
robot_material.SetRestitution(0.1)

# ---------------------------------------------------------------------
# Set up Irrlicht visualization
# ---------------------------------------------------------------------

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Turtlebot Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Add lighting
vis.AddLight(chrono.ChVectorD(5, 5, 5), 10, chrono.ChColor(1, 1, 1))
vis.AddLight(chrono.ChVectorD(-5, 5, 5), 10, chrono.ChColor(0.8, 0.8, 1))

# Set camera settings
vis.GetActiveCamera().setPosition(chrono.vector3df(4, 4, 2))
vis.GetActiveCamera().setTarget(chrono.vector3df(0, 0, 0))

# Enable shadows
vis.EnableShadows()

# ---------------------------------------------------------------------
# Simulation parameters and motor control setup
# ---------------------------------------------------------------------

# Simulation parameters
time_step = 0.01
simulation_time = 20.0  # Total simulation time in seconds
realtime_timer = chrono.ChRealtimeStepTimer()

# Motor control parameters
left_motor_speed = 0.0
right_motor_speed = 0.0
max_motor_speed = 5.0  # rad/s

# Define movement sequence with times and motor speeds
movement_sequence = [
    # (start_time, end_time, left_speed, right_speed, description)
    (0.0, 3.0, 2.0, 2.0, "Move forward"),
    (3.0, 5.0, 1.0, -1.0, "Turn right"),
    (5.0, 8.0, 2.0, 2.0, "Move forward"),
    (8.0, 10.0, -1.0, 1.0, "Turn left"),
    (10.0, 13.0, 2.0, 2.0, "Move forward"),
    (13.0, 15.0, 1.5, -1.5, "Turn right"),
    (15.0, 18.0, 2.0, 2.0, "Move forward"),
    (18.0, 20.0, 0.0, 0.0, "Stop")
]

current_sequence_index = 0

# ---------------------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------------------

print("Starting Turtlebot simulation...")
print("Movement sequence:")
for i, (start, end, left, right, desc) in enumerate(movement_sequence):
    print(f"  {i+1}. {start:4.1f}-{end:4.1f}s: {desc} (L:{left:4.1f}, R:{right:4.1f})")

# Initialize timing
system.SetChTime(0)
time_start = time.time()

while vis.Run() and system.GetChTime() < simulation_time:
    # Get current simulation time
    current_time = system.GetChTime()
    
    # Update motor speeds based on movement sequence
    if current_sequence_index < len(movement_sequence):
        start_time, end_time, left_speed, right_speed, description = movement_sequence[current_sequence_index]
        
        if current_time >= start_time and current_time < end_time:
            left_motor_speed = left_speed
            right_motor_speed = right_speed
        elif current_time >= end_time:
            current_sequence_index += 1
            if current_sequence_index < len(movement_sequence):
                print(f"Time {current_time:.1f}s: Starting - {movement_sequence[current_sequence_index][4]}")
    
    # Apply motor speeds to robot (this is a simplified approach)
    # In a real implementation, you would use the robot's specific motor control API
    try:
        # Apply forces to simulate motor control
        robot_body = robot.GetChassisBody()
        robot_pos = robot_body.GetPos()
        robot_rot = robot_body.GetRot()
        
        # Calculate wheel positions (approximate)
        wheel_separation = 0.16  # Typical Turtlebot wheel separation
        
        # Convert motor speeds to forces (simplified)
        force_scale = 2.0
        left_force = left_motor_speed * force_scale
        right_force = right_motor_speed * force_scale
        
        # Apply differential forces for steering
        forward_force = (left_force + right_force) / 2.0
        turning_torque = (right_force - left_force) * wheel_separation / 2.0
        
        # Apply forward force
        forward_dir = robot_rot.Rotate(chrono.ChVectorD(1, 0, 0))
        robot_body.Accumulate_force(forward_dir * forward_force, robot_pos, False)
        
        # Apply turning torque
        robot_body.Accumulate_torque(chrono.ChVectorD(0, 0, turning_torque), False)
        
    except Exception as e:
        print(f"Motor control error: {e}")
        # Continue with basic motor speeds
        pass
    
    # Update camera to follow robot
    if hasattr(robot, 'GetChassisBody'):
        robot_pos = robot.GetChassisBody().GetPos()
        camera_offset = chrono.ChVectorD(3, 3, 2)
        vis.GetActiveCamera().setPosition(chrono.vector3df(
            robot_pos.x + camera_offset.x,
            robot_pos.y + camera_offset.y,
            robot_pos.z + camera_offset.z
        ))
        vis.GetActiveCamera().setTarget(chrono.vector3df(robot_pos.x, robot_pos.y, robot_pos.z))
    
    # Begin scene rendering
    vis.BeginScene()
    vis.Render()
    
    # Display current information
    info_text = f"Time: {current_time:.1f}s | Left Motor: {left_motor_speed:.1f} | Right Motor: {right_motor_speed:.1f}"
    if current_sequence_index < len(movement_sequence):
        info_text += f" | Action: {movement_sequence[current_sequence_index][4]}"
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        info_text.encode('utf-8'),
        chrono.recti(10, 10, 400, 30),
        chrono.SColor(255, 255, 255, 255)
    )
    
    vis.EndScene()
    
    # Advance the simulation
    system.DoStepDynamics(time_step)
    
    # Real-time synchronization
    realtime_timer.Spin(time_step)

print("Simulation completed!")