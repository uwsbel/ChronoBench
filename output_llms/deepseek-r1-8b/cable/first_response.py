import pychrono as pychrono
import pychrono.viewer.iranviewer as iranviewer

# Initialize PyChrono environment and core components
pychrono.initialize()
scene = pychrono.Scene()
camera = pychrono.Camera()
renderer = iranviewer.IrrRenderer()
renderer.setScene(scene)
renderer.setLightingEnabled(True)

# Define the beam's properties
num_elements = 10
length = 5.0  # Total length of the beam
cross-sectional_area = 0.1  # m²
density = 8000  # kg/m³
young_modulus = 210000  # Pa
poisson_ratio = 0.2

# Create ANCF cable elements
cable_elements = []
for i in range(num_elements):
    cable = pychrono.RigidRod(
        name="cable" + str(i),
        length=length / num_elements,
        cross-sectional_area=cross-sectional_area,
        density=density,
        young_modulus=young_modulus,
        poisson_ratio=poisson_ratio
    )
    cable_elements.append(cable)

# Create nodes
fixed_node = pychrono.Node("fixed", pychrono.Vec3(0, 0, 0), pychrono.Vec3(0, 0, 0))
free_node = pychrono.Node("free", pychrono.Vec3(0, length, 0), pychrono.Vec3(0, 0, 0))

# Link nodes
current_node = fixed_node
for cable in cable_elements:
    current_node = pychrono.RigidRod.create(
        name=cable.name,
        first_node=current_node,
        second_node=pychrono.Node("node" + str(i), pychrono.Vec3(
            0, (i+1)*length/num_elements, 0), pychrono.Vec3(0, 0, 0))
    )

# Apply gravity to the free node
gravity_field = pychrono.Forces.GravityField(g=9.81, direction=pychrono.Vec3(0, -1, 0))
force = pychrono.Forces.Force(
    name="gravity",
    body=free_node,
    force=gravity_field.get_force(free_node.get_pos())
)

# Add elements and forces to the scene
scene.add_elements(cable_elements)
scene.add_forces([force])

# Set up visualization
renderer.setLightDirection(pychrono.Vec3(1, 1, 1))
renderer.setLightIntensity(1)
renderer.setAmbientLight(pychrono.Vec4(0.5, 0.5, 0.5, 1))

# Set camera position and view
camera.set_pos(pychrono.Vec3(5, 5, 5))
camera.look_at(pychrono.Vec3(0, 0, 0))

# Simulation parameters
time_step = 0.01
max_steps = 10000

# Simulation loop
for step in range(max_steps):
    pychrono.Simulate()
    renderer.update()
    
    # Optional: Record node positions for visualization
    # nodes_pos = [node.get_pos() for node in scene.get_nodes()]
    
# Close the viewer
renderer.close()