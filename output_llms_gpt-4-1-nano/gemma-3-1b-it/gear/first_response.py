import pychrono
import pychrono.core
import pychrono.visuals
import pychrono.visuals.materials
import pychrono.visuals.lights
import pychrono.visuals.colors
import pychrono.visuals.objects
import pychrono.visuals.visuals_utils
import pychrono.visuals.visuals
import pychrono.visuals.camera
import pychrono.visuals.perspective
import pychrono.visuals.rotation_control
import pychrono.visuals.rotation_control.rotation_control
import pychrono.visuals.rotation_control.rotation_control_utils
import pychrono.visuals.rotation_control.rotation_control_utils.rotation_control_utils
import pychrono.visuals.rotation_control.rotation_control_utils.rotation_control_utils

# --- Simulation Parameters ---
simulation_name = "Epicyclic Gears"
simulation_time = 10  # seconds
simulation_speed = 1.0  # radians/second
num_gear_motors = 2
gear_ratio = 1.0  # Gear ratio for the system
truss_radius = 0.5
bar_radius = 0.3
gear_motor_speed = 1.0  # radians/second
truss_force = 1000.0  # Newtons
bar_force = 1000.0 # Newtons
# --- System Setup ---

# Create the fixed truss
fixed_truss = pychrono.visuals.objects.Truss(
    radius=truss_radius,
    height=0.1,
    color="lightgray",
    material=pychrono.visuals.materials.Material(
        name="Steel",
        color=(0.8, 0.8, 0.8),
        roughness=0.5
    )
)

# Create the rotating bar
rotating_bar = pychrono.visuals.objects.Bar(
    radius=bar_radius,
    height=0.1,
    color="red",
    material=pychrono.visuals.materials.Material(
        name="Steel",
        color=(0.8, 0.8, 0.8),
        roughness=0.5
    )
)

# Create the two gears with a motor
gear1_motor = pychrono.visuals.objects.Gear(
    radius=gear_motor_speed,
    height=0.1,
    color="blue",
    material=pychrono.visuals.materials.Material(
        name="Steel",
        color=(0.8, 0.8, 0.8),
        roughness=0.5
    )
)
gear2_motor = pychrono.visuals.objects.Gear(
    radius=gear_motor_speed,
    height=0.1,
    color="green",
    material=pychrono.visuals.materials.Material(
        name="Steel",
        color=(0.8, 0.8, 0.8),
        roughness=0.5
    )
)

# --- Visualization Setup ---
# Create the camera
camera = pychrono.visuals.camera.Camera(
    width=100,
    height=100,
    perspective=pychrono.visuals.perspective.Perspective(
        camera_angle=0,
        camera_offset=0,
        camera_scale=1.0
    )
)

# Create the Irrlicht visualization
visual = pychrono.visuals.visuals.Visual(
    camera=camera,
    width=1000,
    height=1000,
    color=pychrono.visuals.colors.Color(0.8, 0.8, 0.8),
    background=pychrono.visuals.colors.Color(0.5, 0.5, 0.5),
    light=pychrono.visuals.lights.Light(
        color=pychrono.visuals.colors.Color(0.5, 0.5, 0.5),
        intensity=1.0
    ),
    material=pychrono.visuals.materials.Material(
        name="Steel",
        color=(0.8, 0.8, 0.8),
        roughness=0.5
    )
)

# --- Simulation Logic ---

# Set initial positions and velocities
fixed_truss.position = (0, 0, 0)
rotating_bar.position = (0, 0, 0)
gear1_motor.position = (0, 0, 0)
gear2_motor.position = (0, 0, 0)

# Set initial velocities
gear1_motor.velocity = (0, 0, 0)
gear2_motor.velocity = (0, 0, 0)

# --- Simulation Loop ---
for i in range(simulation_time):
    # Update positions
    fixed_truss.position = (0, 0, 0)
    rotating_bar.position = (0, 0, 0)
    gear1_motor.position = (0, 0, 0)
    gear2_motor.position = (0, 0, 0)

    # Update velocities
    gear1_motor.velocity = (0, 0, 0)
    gear2_motor.velocity = (0, 0, 0)

    # Visualize the system
    visual.add(fixed_truss)
    visual.add(rotating_bar)
    visual.add(gear1_motor)
    visual.add(gear2_motor)

    # Add a simple rotation control
    if i % 30 == 0:
        visual.rotate(gear1_motor, 1.0)
        visual.rotate(gear2_motor, 1.0)

    # Add a simple force
    visual.add(fixed_truss,  color=pychrono.visuals.colors.Color(0.5, 0.5, 0.5),  material=pychrono.visuals.materials.Material(
        name="Steel",
        color=(0.8, 0.8, 0.8),
        roughness=0.5
    ))
    visual.add(rotating_bar, color=pychrono.visuals.colors.Color(0.8, 0.8, 0.8), material=pychrono.visuals.materials.Material(
        name="Steel",
        color=(0.8, 0.8, 0.8),
        roughness=0.5
    ))
    visual.add(gear1_motor, color=pychrono.visuals.colors.Color(0.8, 0.8, 0.8), material=pychrono.visuals.materials.Material(
        name="Steel",
        color=(0.8, 0.8, 0.8),
        roughness=0.5
    ))
    visual.add(gear2_motor, color=pychrono.visuals.colors.Color(0.8, 0.8, 0.8), material=pychrono.visuals.materials.Material(
        name="Steel",
        color=(0.8, 0.8, 0.8),
        roughness=0.5
    ))

    # Update the simulation time
    simulation_time += 1
    # Optional:  Add a simple animation loop to visualize the system
    # This would require more complex control and potentially a different visualization approach.
    # For now, just display the system.
    # visual.show()

print("Simulation complete.")