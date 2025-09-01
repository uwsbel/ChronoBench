from pychrono import *
from pychrono.gear import *
from pychrono.vis import *


chrono = Ch chrono()
chrono.setgravity([0, -9.81, 0])  
chrono.set_unit_length(1)  
chrono.set_time_unit(chrono.DYNAMIC)  
chrono.set_step(0.01)  


T_total = 5.0  
n_steps = int(T_total / chrono.get_step())  


origin = chrono.Point()
beam_center = origin.translate([0, 0, 0])
beam_end1 = beam_center.translate([0.5, 0, 0])
beam_end2 = beam_center.translate([-0.5, 0, 0])


beam = chrono.Beams()
beam.set_name("IGA Beam")
beam.set_origin(beam_center)
beam.set_length(1.0)  
beam.set_cross_section_type(chrono.Beams.CROSS_SECTION_TYPE_RECTANGULAR)  
beam.set_width(0.02)  
beam.set_height(0.02)  
beam.set_order(4)  
beam.set_material(chrono.Materials.NEW_MATERIAL)
beam.set_young_modulus(200e9)  
beam.set_density(7850)  


flywheel = chrono.RigidDisk()
flywheel.set_name("Flywheel")
flywheel.set_origin(beam_center)
flywheel.set_mass(1)  
flywheel.set_radius(0.5)  
flywheel.set_moment_inertia(chrono.MomentOfInertia.DISC, flywheel.get_mass(), flywheel.get_radius())


motor = chrono.RotationalMotor()
motor.set_name("Motor")
motor.set_origin(beam_end1)
motor.set_torque(10)  
motor.set_rotation_axis(chrono.Axis.X)  


beam_end1.attach(motor)


motor.set_torque_enabled(True)
chrono.add_force(motor, beam_end1)


beam_center.attach(flywheel)


beam.add()


vis = chrono.Vis()
vis.set_window_size(800, 600)
vis.set_window_pos(0, 0, 0)
vis.add_object(beam)
vis.add_object(flywheel)
vis.add_object(motor)
vis.set_window_enabled(True)


chrono.initialize()
chrono.start()


while chrono.get_step() <= n_steps:
    chrono.step()
    vis.update()


chrono.stop()
chrono finalized