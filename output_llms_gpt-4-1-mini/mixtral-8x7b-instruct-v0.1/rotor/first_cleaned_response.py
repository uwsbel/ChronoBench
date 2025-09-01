from pychrono.core import *
from pychrono.irrlicht import *
from pychrono.fea import *


my_system = ChronoSystem()


my_visualization = IrrlichtVisualization(my_system, "Jeffcott Rotor IGA", 800, 600, false)
my_visualization.set_background_color(0.4, 0.4, 0.8)
my_visualization.set_camera_position(Vector3(5, 5, 5))
my_visualization.set_camera_looking_at(Vector3(0, 0, 0))




beam_shape = IGABeamGeometry()
beam_shape.set_width(0.1)
beam_shape.set_height(0.02)
beam_shape.set_length(1)
beam_shape.set_num_segments(10)
beam_shape.set_num_elements(100)
beam_shape.update_geometry()

beam_material = StructuralSteel()
beam_material.set_density(7800)
beam_material.set_young_modulus(210e9)
beam_material.set_poisson_ratio(0.3)

beam_body = RigidBody(my_system, "beam_body")
beam_spatial = IGA_Beam(beam_body, beam_shape, beam_material)
beam_spatial.set_initial_position(ChVector<>(0, 0, 0))




flywheel_shape = ChBoxShape()
flywheel_shape.get_box_geometry().size = ChVector<>(0.2, 0.05, 0.05)

flywheel_material = StructuralSteel()
flywheel_material.set_density(7800)

flywheel_body = RigidBody(my_system, "flywheel_body")
flywheel_body.add_shape(flywheel_shape)
flywheel_body.set_mass(1)
flywheel_body.set_pos(ChVector<>(0, 0, 0.05))
beam_spatial.add_child_body(flywheel_body)




rotational_motor = RotationalMotor(my_system, "rotational_motor")
rotational_motor.set_axis(ChVector3(0, 0, 1))
rotational_motor.set_joint_target_velocity(10)
beam_spatial.add_motor(rotational_motor)




my_visualization.attach_object(beam_spatial.get_fea_visualization())




my_system.set_solver_type(ChSolverType.CHRONO_SOLVER_ITERATIVE_NEWTON)
my_system.set_solver_max_iterations(100)
my_system.set_solver_tolerance(1e-6)
my_system.set_timestepper_type(ChTimestepperType.HHT)
my_system.set_step_method(ChTimestepperHHT::Type.EULER_IMPLICIT_LINEARIZED)
my_system.set_time_step(1e-4)

for i in range(1000):
    my_system.do_step()
    my_visualization.run_one_timestep()


my_visualization.wait_for_user()
my_visualization.close()