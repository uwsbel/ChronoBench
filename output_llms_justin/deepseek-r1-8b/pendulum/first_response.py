import pychrono as pychrono
import pychrono.trajectory as trajectory
import pychronovisualization.irdock as irdock
import pychrono.dynamics as dynamics
import pychrono.gear as gear
import pychrono.contact as contact
import pychrono.math as math

# Initialize PyChrono and visualization
pychrono.init()
sim = pychrono.Simulation()
sim.set_gfr(True)
sim.set CONTACT_RESOLVER(True)
sim.set_contact_resolver(contact.ContactResolver())

# Create ground body
ground_body = pychrono.Body()
ground_body.set_name("ground")
ground_body.set_shape(pychrono.Shape.SPHERE)
ground_body.set_mass(0)
ground_body.set_position(pychrono.Vec3(0, 0, 0))
ground_body.set_inertia(dynamics.InertiaCube(mass=0, arm1=0, arm2=0, arm3=0))
sim.add_body(ground_body)

# Create pendulum body
pendulum_mass = 1.0
length = 1.0
moments_of_inertia = dynamics.InertiaCube(mass=pendulum_mass, arm1=length, arm2=length, arm3=0)
pendulum_body = pychrono.Body()
pendulum_body.set_name("pendulum")
pendulum_body.set_shape(pychrono.Shape.SPHERE)
pendulum_body.set_mass(pendulum_mass)
pendulum_body.set_position(pychrono.Vec3(0, 0, length))
pendulum_body.set_inertia(moments_of_inertia)
sim.add_body(pendulum_body)

# Create revolute joint between pendulum and ground
joint = pychrono.Joint()
joint.set_name("revolute")
joint.set_parent_body(ground_body)
joint.set_child_body(pendulum_body)
joint.set_type(pychrono.Joint.ROTATION)
joint.set_axis(pychrono.Vec3(0, 0, 1))
sim.add_joint(joint)

# Set up visualization
renderer = irdock.IrrlichtRenderer()
renderer.set_camera_position(pychrono.Vec3(5, 5, 5))
renderer.set_background_color(pychrono.Color(0, 0, 0))
renderer.set_light_color(pychrono.Color(1, 1, 1))
renderer.set_light_position(pychrono.Vec3(5, 5, 5))
sim.add_renderer(renderer)

# Define forces and interactions
spring_force = dynamics.SpringForce()
spring_force.set_name("spring")
spring_force.set_type(dynamics.SpringForce.HARMONIC_POTENTIAL)
spring_force.set_stiffness(100.0)
spring_force.set_body1(pendulum_body)
spring_force.set_body2(ground_body)
spring_force.set_origin(ground_body.get_position())
spring_force.set_target_position(ground_body.get_position())
sim.add_force(spring_force)

# Log positions and velocities
log = pychrono.Logger("pendulum")
log.set_period(0.1)
log.add(pendulum_body)

# Simulation setup
sim.set_time_step(0.001)
sim.set_number_of_steps(1000)

# Function to log data
def log_data():
    log.log("Time: %.2f, Position: %.12f, Velocity: %.12f" % (
        sim.get_time(),
        pendulum_body.get_position().x,
        pendulum_body.get_velocity().x
    ))

# Start simulation
sim.start()
sim.run()