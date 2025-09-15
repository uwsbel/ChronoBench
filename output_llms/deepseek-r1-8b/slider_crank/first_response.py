import PyChrono as pc
from pychrono.objects import *
from pychrono.visualization import *

# Initialize PyChrono simulation
sim = pc.Simulation()
sim.set_tolerance(1e-8)

# Define system components
# Floor/truss
floor = Box2D(
    name="Floor",
    size=(2, 2, 0.2),
    position=(0, 0, 0),
    material=Material(
        color=(0.8, 0.2, 0.2),
        texture="floor",
        double_sided=False
    )
)
sim.add_object(floor)

# Crankshaft
crankshaft = Rod2D(
    name="Crankshaft",
    length=0.5,
    cross_section=0.01,
    position=(0, 0, 0),
    orientation=(0, 0, 1),
    material=Material(
        color=(0.9, 0.9, 0.9),
        texture="aluminum",
        double_sided=False
    )
)
sim.add_object(crankshaft)

# Connecting rod
connecting_rod = Rod2D(
    name="ConnectingRod",
    length=0.4,
    cross_section=0.01,
    position=crankshaft.position + (crankshaft.length, 0, 0),
    orientation=(0, 0, 1),
    joints=Joint2D(
        type=Joint2D.Pin,
        first_body=crankshaft,
        second_body=floor,
        position=(0, 0, 0)
    ),
    joints=Joint2D(
        type=Joint2D.Spherical,
        first_body=crankshaft,
        second_body=floor,
        position=(0, 0, 0)
    ),
    material=Material(
        color=(0.9, 0.9, 0.9),
        texture="aluminum",
        double_sided=False
    )
)
sim.add_object(connecting_rod)

# Piston
piston = Cylinder2D(
    name="Piston",
    radius=0.05,
    height=0.1,
    position=connecting_rod.position + (0, 0, 0.1),
    orientation=(0, 0, 1),
    material=Material(
        color=(0, 0, 0),
        texture="piston",
        double_sided=False
    )
)
sim.add_object(piston)

# Piston to floor joint
piston_to_floor = Joint2D(
    type=Joint2D.Slider,
    first_body=piston,
    second_body=floor,
    position=(0, 0, 0.1 - 0.05)
)

# Motor
motor = RotationalMotor2D(
    name="Motor",
    angular_speed=60,  # Degrees per second
    axis=(0, 0, 1),
    connected_body=crankshaft,
    connected_joint=crankshaft.joints[0]
)
sim.add_object(motor)

# Visualization setup
# Create visualization
viz = Visualization(sim)
viz.camera = Camera2D(
    position=(5, 5, 5),
    look_at=(0, 0, 0),
    up=(0, 1, 0),
    fov=60,
    size=(800, 600)
)

# Add lighting
viz.add_light(
    position=(10, 10, 10),
    color=(1, 1, 1)
)

# Add textures
viz.textures.add("floor", "pychrono/textures/floor")
viz.textures.add("aluminum", "pychrono/textures/aluminum")
viz.textures.add("piston", "pychrono/textures/piston")

# Setup simulation
sim.init()

# Simulation loop
while True:
    # Update motor
    motor.update()
    
    # Update simulation
    sim.update()
    
    # Check contacts and joints
    sim.check_contacts()
    sim.check_joints()
    
    # Visualize
    viz.render()
    
    # Close loop when window is closed
    if viz.window.closed:
        break