# =============================================================================
# PyChrono Simulation: Curiosity Rover on Rigid Terrain
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.robot as robot
import math

# =============================================================================
# Simulation Parameters
# =============================================================================
step_size = 1e-3          # Simulation time step (seconds)
total_time = 30.0         # Total simulation duration (seconds)
render_step_size = 1.0 / 50  # Rendering step size (50 FPS)

# Terrain dimensions
terrain_length = 20.0
terrain_width = 20.0

# Rover initial position and orientation
init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)

# =============================================================================
# Initialize Chrono System
# =============================================================================
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Set solver parameters
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetSolverForceTolerance(1e-10)

# Set collision system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# =============================================================================
# Create Ground / Terrain
# =============================================================================
# Ground material
ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.01)

# Create ground body
ground = chrono.ChBodyEasyBox(
    terrain_length, 0.5, terrain_width,  # dimensions
    1000,                                  # density
    True,                                  # create collision shape
    True,                                  # create visualization shape
    ground_mat
)
ground.SetPos(chrono.ChVectorD(0, -0.25, 0))
ground.SetBodyFixed(True)
ground.GetName()
ground.SetName("Ground")

# Ground visual - texture
ground_visual = ground.GetVisualShape(0)
if ground_visual:
    ground_texture = chrono.ChVisualMaterial()
    ground_texture.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground_texture.SetSpecularColor(chrono.ChColor(0.2, 0.2, 0.2))
    ground_visual.AddMaterial(ground_texture)
    ground_visual.SetTextureScale(4.0, 4.0)

system.Add(ground)

# =============================================================================
# Add Some Obstacles / Rocks on Terrain
# =============================================================================
rock_mat = chrono.ChMaterialSurfaceNSC()
rock_mat.SetFriction(0.8)
rock_mat.SetRestitution(0.02)

rock_positions = [
    chrono.ChVectorD(3, 0.15, 1),
    chrono.ChVectorD(-2, 0.1, 3),
    chrono.ChVectorD(5, 0.12, -2),
    chrono.ChVectorD(-4, 0.18, -1),
    chrono.ChVectorD(1, 0.1, -4),
]
rock_sizes = [0.3, 0.2, 0.25, 0.35, 0.15]

for i, (pos, size) in enumerate(zip(rock_positions, rock_sizes)):
    rock = chrono.ChBodyEasySphere(size, 2500, True, True, rock_mat)
    rock.SetPos(pos)
    rock.SetBodyFixed(True)
    rock.SetName(f"Rock_{i}")
    
    rock_visual = rock.GetVisualShape(0)
    if rock_visual:
        rock_mat_vis = chrono.ChVisualMaterial()
        rock_mat_vis.SetKdTexture(chrono.GetChronoDataFile("textures/rock.jpg"))
        rock_visual.AddMaterial(rock_mat_vis)
    
    system.Add(rock)

# =============================================================================
# Create Curiosity Rover
# =============================================================================
# Create the Curiosity rover using PyChrono robot module
curiosity = robot.Curiosity(system)

# Create a motor driver for the rover
driver = robot.CuriositySpeedDriver(1.0, 1.0)  # ramp_time, max_speed
curiosity.SetDriver(driver)

# Initialize rover position and orientation
curiosity.Initialize(chrono.ChFrameD(init_pos, init_rot))

print("Curiosity rover initialized successfully!")
print(f"Rover position: {curiosity.GetChassisPos()}")

# =============================================================================
# Irrlicht Visualization Setup
# =============================================================================
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle("Curiosity Rover on Rigid Terrain - PyChrono")
vis.SetWindowSize(1280, 720)

# Camera settings
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVectorD(-5, 3, -5),  # Camera position
    chrono.ChVectorD(0, 0, 0)      # Camera target
)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)

# Lighting
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVectorD(10, 20, 10),   # Light position
    chrono.ChVectorD(0, 0, 0),       # Light target
    40,                               # Radius
    1, 60,                            # Near/far clip
    50                                # Angle
)
vis.EnableShadows()

# Add logo
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))

# =============================================================================
# Simulation Control Variables
# =============================================================================
steering_angle = 0.0          # Current steering angle
target_speed = 1.5            # Target rover speed (m/s)
time = 0.0
render_steps = int(render_step_size / step_size)
step_number = 0

print("\n=== Simulation Controls ===")
print("The rover will automatically navigate with pre-programmed maneuvers.")
print("Watch the Curiosity rover traverse the terrain!\n")

# =============================================================================
# Pre-programmed maneuver sequence
# =============================================================================
def get_steering_and_speed(t):
    """
    Returns (steering, speed) based on simulation time.
    Creates an interesting navigation pattern.
    """
    if t < 3.0:
        # Move straight forward
        return 0.0, 1.5
    elif t < 6.0:
        # Gentle left turn
        return 0.3, 1.2
    elif t < 9.0:
        # Straighten out
        return 0.0, 1.5
    elif t < 12.0:
        # Right turn
        return -0.4, 1.0
    elif t < 15.0:
        # Move straight
        return 0.0, 1.5
    elif t < 18.0:
        # S-curve: left
        return 0.25, 1.2
    elif t < 21.0:
        # S-curve: right
        return -0.25, 1.2
    elif t < 24.0:
        # Slow down and turn
        return 0.5, 0.8
    else:
        # Final straight
        return 0.0, 1.5

# =============================================================================
# Main Simulation Loop
# =============================================================================
print("Starting simulation...")

while vis.Run() and time < total_time:
    
    # Get current steering and speed commands
    steering, speed = get_steering_and_speed(time)
    
    # Update driver inputs
    driver.SetSteering(steering)
    
    # Render at specified frequency
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        
        # Draw real-time info overlay
        irr.drawAllContactPoints(vis, 1.5, irr.IrrContactsDrawMode_CONTACT_DISTANCES)
        
        vis.EndScene()
        
        # Print status every 100 render steps
        if step_number % (render_steps * 100) == 0:
            rover_pos = curiosity.GetChassisPos()
            print(f"Time: {time:.2f}s | "
                  f"Pos: ({rover_pos.x:.2f}, {rover_pos.y:.2f}, {rover_pos.z:.2f}) | "
                  f"Steering: {steering:.2f} | Speed: {speed:.2f}")
    
    # Update rover driver
    curiosity.Update()
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    
    time += step_size
    step_number += 1

print("\nSimulation completed!")
print(f"Final rover position: {curiosity.GetChassisPos()}")