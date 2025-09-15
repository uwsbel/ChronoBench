import pychrono as pc
pc.init()


simulation = pc.Simulation()
viewer = pc.viewer.Viewer()
viewer.init()


L = 5.0  
A = 0.1  
E = 20000  
nu = 0.2  
density = 8000  


beam = pc_beam.Beam()
beam.set_length(L)
beam.set_cross_section(A, E, nu)
beam.set_density(density)
beam.set_position(pc.Vec3(0, 0, 0))
beam.set_orientation(pc.Vec3(1, 0, 0))


weight = pc.rigidbody.RigidBody()
weight.set_mass(1)
weight.set_position(pc.Vec3(L, -0.5, 0))
weight.set_size(pc.Vec3(0.1, 0.1, 0.1))
beam.set_restitution(0.5)
beam.set_friction(0.5)


ground = pc.contact.ContactGround()
beam.add_contact_ground(ground)
weight.add_contact_ground(ground)


motor = pc.motor.Motor()
motor.set_name("Buckling Motor")
motor.set_type(pc.motor.MotorType.ROTARY)
motor.set_position(weight.get_position())
motor.set_force_func(lambda t: 1000 * (1 - (t / 50.0)))


def motor_force(t):
    return 1000 * (1 - (t / 50.0))


weight.add_constraint(motor)


constraint_beam = pc.constraint.Constraint()
constraint_beam.set_type(pc.constraint.ConstraintType.HINGE)
constraint_beam.set_parent1(beam)
constraint_beam.set_parent2(pc.Vec3(0, 0, 0))
beam.add_constraint(constraint_beam)

constraint_weight = pc.constraint.Constraint()
constraint_weight.set_type(pc.constraint.ConstraintType.RIGID)
constraint_weight.set_parent1(weight)
constraint_weight.set_parent2(beam)
constraint_weight.set_axis(pc.Vec3(0, 1, 0))
beam.add_constraint(constraint_weight)


viewer.set_camera_position(pc.Vec3(2, 2, 2))
viewer.set_lookat(pc.Vec3(0, 0, 0))
viewer.set_up(pc.Vec3(0, 0, 1))
viewer.add_light(pc.Vec3(5, 5, 5), pc.Vec3(0, 0, 0))
viewer.add_light(pc.Vec3(-5, 5, 5), pc.Vec3(0, 0, 0))


simulation.set_solver(pc.solver.Solver())
simulation.set_timestep(0.001)
simulation.set_max_steps(100000)


simulation.set_gravity(pc.Vec3(0, -9.81, 0))
simulation.set_damping(0.1)
simulation.set_record(True)
simulation.set_record_path("results/")


simulation.start()


import pychrono as pc
pc.init()


simulation = pc.Simulation()
viewer = pc.viewer.Viewer()
viewer.init()


L = 5.0  
A = 0.1  
E = 20000  
nu = 0.2  
density = 8000  


beam = pc_beam.Beam()
beam.set_length(L)
beam.set_cross_section(A, E, nu)
beam.set_density(density)
beam.set_position(pc.Vec3(0, 0, 0))
beam.set_orientation(pc.Vec3(1, 0, 0))


weight = pc.rigidbody.RigidBody()
weight.set_mass(1)
weight.set_position(pc.Vec3(L, -0.5, 0))
weight.set_size(pc.Vec3(0.1, 0.1, 0.1))
beam.set_restitution(0.5)
beam.set_friction(0.5)


ground = pc.contact.ContactGround()
beam.add_contact_ground(ground)
weight.add_contact_ground(ground)


motor = pc.motor.Motor()
motor.set_name("Buckling Motor")
motor.set_type(pc.motor.MotorType.ROTARY)
motor.set_position(weight.get_position())
motor.set_force_func(lambda t: 1000 * (1 - (t / 50.0)))


def motor_force(t):
    return 1000 * (1 - (t / 50.0))


weight.add_constraint(motor)


constraint_beam = pc.constraint.Constraint()
constraint_beam.set_type(pc.constraint.ConstraintType.HINGE)
constraint_beam.set_parent1(beam)
constraint_beam.set_parent2(pc.Vec3(0, 0, 0))
beam.add_constraint(constraint_beam)

constraint_weight = pc.constraint.Constraint()
constraint_weight.set_type(pc.constraint.ConstraintType.RIGID)
constraint_weight.set_parent1(weight)
constraint_weight.set_parent2(beam)
constraint_weight.set_axis(pc.Vec3(0, 1, 0))
beam.add_constraint(constraint_weight)


viewer.set_camera_position(pc.Vec3(2, 2, 2))
viewer.set_lookat(pc.Vec3(0, 0, 0))
viewer.set_up(pc.Vec3(0, 0, 1))
viewer.add_light(pc.Vec3(5, 5, 5), pc.Vec3(0, 0, 0))
viewer.add_light(pc.Vec3(-5, 5, 5), pc.Vec3(0, 0, 0))


simulation.set_solver(pc.solver.Solver())
simulation.set_timestep(0.001)
simulation.set_max_steps(100000)


simulation.set_gravity(pc.Vec3(0, -9.81, 0))
simulation.set_damping(0.1)
simulation.set_record(True)
simulation.set_record_path("results/")


simulation.start()