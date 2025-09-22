import pychrono as pychrono
import pychrono_dgp as dgp
import pychrono_irrlight as irllight
import pychrono_viewer as viewer
import pyplot as plt


pychrono.init()


pychrono.set_gravity(9.81)
pychrono.set Visualization( enable=irllight.ENABLE, camera=(0, 0, 5, 0, 0, 0), lighting=(0, 0, 5, 0, 0, 0) )


ground = pychrono.Body()
ground.set_mass(1000)
ground.set_position((0, 0, 0))
ground.set_damping(0.01)
ground.set_type(pychrono.BodyType.STATIC)
chrono.systems.append(ground)


turtlebot = pychrono.Body()
turtlebot.set_mass(2)
turtlebot.set_position((0, 0.5, 0))
turtlebot.set_orientation(pychrono.Vector3(0, 0, 0))  
chrono.systems.append(turtlebot)


wheel_left = pychrono.Body()
wheel_left.set_mass(0.1)
wheel_left.set_position((0.2, 0.5, 0))
wheel_left.set_radius(0.1)
wheel_left.set_type(pychrono.BodyType.DYNAMIC)
chrono.systems.append(wheel_left)

wheel_right = pychrono.Body()
wheel_right.set_mass(0.1)
wheel_right.set_position((-0.2, 0.5, 0))
wheel_right.set_radius(0.1)
wheel_right.set_type(pychrono.BodyType.DYNAMIC)
chrono.systems.append(wheel_right)


joint_left = pychrono.Joint()
joint_left.set_type(pychrono.JointType.HINGE)
joint_left.set_parent(turtlebot)
joint_left.set_child(wheel_left)
joint_left.set_position((0.2, 0.5, 0))
chrono.systems.append(joint_left)

joint_right = pychrono.Joint()
joint_right.set_type(pychrono.JointType.HINGE)
joint_right.set_parent(turtlebot)
joint_right.set_child(wheel_right)
joint_right.set_position((-0.2, 0.5, 0))
chrono.systems.append(joint_right)


timestep = 0.01
dt = pychrono.get_time_step()
simulation_loop = pychrono.Simulation(timestep)


left_motor = pychrono.Motor()
right_motor = pychrono.Motor()


plt.figure(figsize=(8, 6))
plt.show()


while True:
    
    t = pychrono.get_time()
    
    
    
    
    left_motor.set_angle(turtlebot, 0)
    right_motor.set_angle(turtlebot, 0)
    
    
    viewer.update()
    plt.draw()
    plt.show()
    
    
    simulation_loop.step()