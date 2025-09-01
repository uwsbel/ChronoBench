import pychrono
import pychrono.visuals as visuals
import pychrono.physics as physics
import pychrono.objects as objects
import math

# 1. Environment Setup & Core Components
# Define the physical system
system = objects.System()
system.add_object(objects.CrankSlider(
    x=0.0,
    y=0.0,
    z=0.0,
    rotation=0.0,  # Initial rotation
    name="CrankSlider",
    force_applied=0.0, # Initial force
    mass=1.0,
    friction=0.0
))

# Define the floor (truss)
floor = objects.Floor()
floor.name = "Floor"
floor.add_object(objects.CrankSlider(
    x=0.0,
    y=0.0,
    z=0.0,
    rotation=0.0,
    name="Floor",
    force_applied=0.0,
    mass=1.0,
    friction=0.0
))

# Define the crankshaft
crank = objects.Crank(
    name="Crank",
    x=0.0,
    y=0.0,
    z=0.0,
    rotation=0.0,
    force_applied=0.0,
    mass=1.0,
    friction=0.0
)

# Define the connecting rod
rod = objects.Rod(
    name="ConnectingRod",
    x=0.0,
    y=0.0,
    z=0.0,
    rotation=0.0,
    force_applied=0.0,
    mass=1.0,
    friction=0.0
)

# Define the piston
piston = objects.Piston(
    name="Piston",
    x=0.0,
    y=0.0,
    z=0.0,
    rotation=0.0,
    force_applied=0.0,
    mass=1.0,
    friction=0.0
)

# 2. Default Parameters (Important!)
# These are set as default values.  You can adjust them.
system.set_default_forces(0.0)  # Initial force on the crankshaft
system.set_default_friction(0.0) # Initial friction on the connecting rod
system.set_default_angle(0.0) # Initial rotation of the crankshaft

# 3. Simulation Setup
# Define the simulation parameters
simulation_time = 10.0  # Simulation duration in seconds
time_step = 0.01 # Time step for calculations

# 4.  Visualization
# Create the Irrlicht scene
visuals.create_scene(
    title="Crank-Slider Simulation",
    width=800,
    height=600,
    background="black")

# Create the floor
floor_object = visuals.Plane(
    name="Floor",
    x=0.0,
    y=0.0,
    z=0.0,
    width=100.0,
    height=100.0,
    color="white"
)
visuals.add_object(floor_object)

# Create the crankshaft
crank_object = visuals.Cube(
    name="Crank",
    x=0.0,
    y=0.0,
    z=0.0,
    width=50.0,
    height=50.0,
    color="red"
)
visuals.add_object(crank_object)

# Create the connecting rod
rod_object = visuals.Cube(
    name="ConnectingRod",
    x=0.0,
    y=0.0,
    z=0.0,
    width=50.0,
    height=50.0,
    color="blue"
)
visuals.add_object(rod_object)

# Create the piston
piston_object = visuals.Cube(
    name="Piston",
    x=0.0,
    y=0.0,
    z=0.0,
    width=50.0,
    height=50.0,
    color="green"
)
visuals.add_object(piston_object)

# 5.  Physics Simulation
# Set the physics system
physics.set_physics_system(system)

# 6.  Simulation Loop
# Run the simulation
try:
    while True:
        # Update the simulation
        system.update()

        # Calculate the forces
        # Calculate the torque on the connecting rod
        torque = (crank.force_applied * rod.mass) / rod.friction

        # Calculate the force on the piston
        piston.force_applied = torque

        # Calculate the angular velocity of the crankshaft
        angular_velocity = crank.rotation

        # Update the crankshaft's rotation
        crank.rotation = angular_velocity

        # Update the connections
        crank.force_applied = torque
        rod.force_applied = torque
        piston.force_applied = torque

        # Update the floor's position
        floor.x = floor.x + 0.1
        floor.y = floor.y + 0.1
        floor.z = floor.z + 0.1


        # Render the scene
        visuals.render(visuals.get_scene())

        # Delay for a short time
        time.sleep(time_step)

except KeyboardInterrupt:
    print("Simulation stopped.")
    break