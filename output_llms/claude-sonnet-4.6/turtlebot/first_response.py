import pychrono as chrono
import pychrono.robot as turtlebot
try:
    import pychrono.irrlicht as chronoirr
except:
    print("Irrlicht module not available")
    exit()

# =============================================================================
# Initialize the Chrono system
# =============================================================================
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Set collision system type
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# =============================================================================
# Create the ground/terrain
# =============================================================================
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.5)
ground_mat.SetRestitution(0.01)

ground = chrono.ChBodyEasyBox(
    20, 20, 0.1,   # dimensions: 20x20x0.1 meters
    1000,          # density
    True,          # create collision shape
    True,          # create visualization shape
    ground_mat     # contact material
)
ground.SetPos(chrono.ChVector3d(0, 0, -0.05))  # position ground so top surface is at z=0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# =============================================================================
# Initialize the Turtlebot robot
# =============================================================================
# Initial position and orientation
init_pos = chrono.ChVector3d(0, 0, 0.2)   # slightly above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity rotation (facing +X direction)

robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# =============================================================================
# Create Irrlicht visualization
# =============================================================================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Turtlebot Simulation on Rigid Terrain")
vis.Initialize()

# Add default logo, sky, lights
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0, -3, 2),   # camera position
    chrono.ChVector3d(0, 0, 0)     # camera look-at point
)
vis.AddTypicalLights()
vis.AddLight(
    chrono.ChVector3d(3, -3, 3),   # light position
    5,                              # light radius
    chrono.ChColor(0.8, 0.8, 0.8)  # light color
)

# =============================================================================
# Simulation parameters
# =============================================================================
time_step = 1e-3   # 1 ms timestep
time_end  = 20.0   # total simulation time in seconds

# Motor speed values (rad/s)
FULL_SPEED   =  5.0   # forward speed
LEFT_SPEED   =  2.0   # speed for left turn (right motor faster)
RIGHT_SPEED  = -2.0   # speed for right turn (left motor faster)
STOP         =  0.0

# Motor control schedule: (start_time, end_time, left_speed, right_speed)
motor_schedule = [
    (0.0,  5.0,  FULL_SPEED,  FULL_SPEED),   # 0-5s: go straight
    (5.0,  8.0,  LEFT_SPEED,  FULL_SPEED),   # 5-8s: turn left
    (8.0, 13.0,  FULL_SPEED,  FULL_SPEED),   # 8-13s: go straight
    (13.0, 16.0, FULL_SPEED,  RIGHT_SPEED),  # 13-16s: turn right
    (16.0, 20.0, FULL_SPEED,  FULL_SPEED),   # 16-20s: go straight
]

def get_motor_speeds(t):
    """Return (left_speed, right_speed) based on simulation time."""
    for (t_start, t_end, left, right) in motor_schedule:
        if t_start <= t < t_end:
            return left, right
    return STOP, STOP

# =============================================================================
# Simulation loop
# =============================================================================
time = 0.0

print("Starting Turtlebot simulation...")
print("Controls:")
print("  0-5s:   Move straight")
print("  5-8s:   Turn left")
print("  8-13s:  Move straight")
print("  13-16s: Turn right")
print("  16-20s: Move straight")

while vis.Run() and time < time_end:
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Get current motor speeds based on time
    left_speed, right_speed = get_motor_speeds(time)

    # Set motor speeds on Turtlebot
    # TurtleBot uses left/right wheel drive
    robot.SetMotorSpeed(left_speed,  turtlebot.TurtleBot.LEFT)
    robot.SetMotorSpeed(right_speed, turtlebot.TurtleBot.RIGHT)

    # Advance the robot (update internal state)
    robot.Update()

    # Advance the system dynamics
    system.DoStepDynamics(time_step)

    time += time_step

    # Optional: print status every second
    if abs(time % 1.0) < time_step:
        pos = robot.GetChassisBody().GetPos()
        print(f"t={time:.1f}s | pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
              f"| motors: L={left_speed:.1f}, R={right_speed:.1f} rad/s")

print("Simulation complete.")