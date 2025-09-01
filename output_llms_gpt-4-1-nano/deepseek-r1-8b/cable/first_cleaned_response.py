import pychrono as pychrono
     import matplotlib.pyplot as plt
     import cv2  

pychrono.init_pychrono()

num_elements = 5
     length = 1.0
     cross_section_area = 0.01
     young_modulus = 100000.0
     tension_strength = 1000.0

elements = []
     for i in range(num_elements):
         elem = pychrono.analyze.ANCFBeamElement(length, cross_section_area, young_modulus, tension_strength)
         elements.append(elem)

hinge = pychrono.rigidbody.JointRevolute()
     free_joint = pychrono.rigidbody.JointFree()

subjects = []
     subjects.append(elements[0])
     subjects.append(elements[1])
     
     subjects.append(hinge)
     subjects.append(free_joint)
     
     ground = pychrono.subjects.Subject()
     ground.add_object(pychrono.objects.StaticPlane())
     subjects.append(ground)

ground.fix(hinge)

gravity = pychrono.forces.Gravity()
     gravity.set_force(free_joint, 0, -9.81, 0)

free_end_node = elements[0].nodes[-1]
     free_end_node.set_position(free_joint, 0, length, 0)

solver = pychrono.solver.Solver()
     for subject in subjects:
         subject.register(solver)

renderer = pychrono.renderer.IrrlichtRenderer()
     camera = pychrono.renderer.Camera(2560, 1440, 500, 1000, 0, 0, 0)
     renderer.set_camera(camera)
     renderer.set_light(0, 0, 1000, 1000, 1000, 0)

time = 0.0
     step = 0.001
     max_steps = 1000

for _ in range(max_steps):
         
         solver.solve()
         
         for elem in elements:
             elem.nodes.update()
         
         renderer.render()
         plt.show()

import pychrono as pychrono
import matplotlib.pyplot as plt
import cv2


pychrono.init_pychrono()


num_elements = 5
length = 1.0
cross_section_area = 0.01
young_modulus = 100000.0
tension_strength = 1000.0


elements = []
for i in range(num_elements):
    elem = pychrono.analyze.ANCFBeamElement(length, cross_section_area, young_modulus, tension_strength)
    elements.append(elem)


hinge = pychrono.rigidbody.JointRevolute()
free_joint = pychrono.rigidbody.JointFree()


subjects = []
for elem in elements:
    subjects.append(elem)
subjects.append(hinge)
subjects.append(free_joint)


ground = pychrono.subjects.Subject()
ground.add_object(pychrono.objects.StaticPlane())
subjects.append(ground)


ground.fix(hinge)


gravity = pychrono.forces.Gravity()
gravity.set_force(free_joint, 0, -9.81, 0)


free_end_node = elements[0].nodes[-1]
free_end_node.set_position(free_joint, 0, length, 0)


solver = pychrono.solver.Solver()
for subject in subjects:
    subject.register(solver)


renderer = pychrono.renderer.IrrlichtRenderer()
camera = pychrono.renderer.Camera(2560, 1440, 500, 1000, 0, 0, 0)
renderer.set_camera(camera)
renderer.set_light(0, 0, 1000, 1000, 1000, 0)


time = 0.0
step = 0.001
max_steps = 1000


for _ in range(max_steps):
    
    solver.solve()
    
    for elem in elements:
        elem.nodes.update()
    
    renderer.render()
    plt.show()


plt.close()