import pychrono as pychrono
import pychrono.viewer.iranviewer as iranviewer


pychrono.initialize()
scene = pychrono.Scene()
camera = pychrono.Camera()
renderer = iranviewer.IrrRenderer()
renderer.setScene(scene)
renderer.setLightingEnabled(True)


num_elements = 10
length = 5.0  
cross-sectional_area = 0.1  
density = 8000  
young_modulus = 210000  
poisson_ratio = 0.2


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


fixed_node = pychrono.Node("fixed", pychrono.Vec3(0, 0, 0), pychrono.Vec3(0, 0, 0))
free_node = pychrono.Node("free", pychrono.Vec3(0, length, 0), pychrono.Vec3(0, 0, 0))


current_node = fixed_node
for cable in cable_elements:
    current_node = pychrono.RigidRod.create(
        name=cable.name,
        first_node=current_node,
        second_node=pychrono.Node("node" + str(i), pychrono.Vec3(
            0, (i+1)*length/num_elements, 0), pychrono.Vec3(0, 0, 0))
    )


gravity_field = pychrono.Forces.GravityField(g=9.81, direction=pychrono.Vec3(0, -1, 0))
force = pychrono.Forces.Force(
    name="gravity",
    body=free_node,
    force=gravity_field.get_force(free_node.get_pos())
)


scene.add_elements(cable_elements)
scene.add_forces([force])


renderer.setLightDirection(pychrono.Vec3(1, 1, 1))
renderer.setLightIntensity(1)
renderer.setAmbientLight(pychrono.Vec4(0.5, 0.5, 0.5, 1))


camera.set_pos(pychrono.Vec3(5, 5, 5))
camera.look_at(pychrono.Vec3(0, 0, 0))


time_step = 0.01
max_steps = 10000


for step in range(max_steps):
    pychrono.Simulate()
    renderer.update()
    
    
    
    

renderer.close()