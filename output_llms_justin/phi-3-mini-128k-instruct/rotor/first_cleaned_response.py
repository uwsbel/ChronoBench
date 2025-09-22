from pychrono.core import *
from pychrono.vehicle_beam import *
from pychrono.vehicle_flywheel import *
from pychrono.vehicle_motor import *
from pychrono.vehicle_ig_beam import *
from pychrono.vehicle_irrlicht_viewer import *
from pychrono.vehicle_ig_beam import *
from pychrono.vehicle_motor_rotary import *


chrono_init()


iga_beam = IGA_Beam()
iga_beam.set_length(1.0)  
iga_beam.set_mass(10.0)  
iga_beam.set_radius(0.05)  
iga_beam.set_elasticity_modulus(210e9)  
iga_beam.set_poisson_ratio(0.3)  
iga_beam.set_density(7800)  
iga_beam.set_damping(0.01)  
iga_beam.set_axis_orientation(CH_AXIS_X)  


flywheel = IGA_Flywheel()
flywheel.set_radius(0.1)  
flywheel.set_mass(5.0)  
flywheel.set_moment_of_inertia(0.0005)  
flywheel.set_angular_velocity(0.0)  
flywheel.set_angular_acceleration(0.0)  


rotary_motor = RotaryMotor()
rotary_motor.set_torque(10.0)  
rotary_motor.set_position(0.5)  


beam_with_flywheel = BeamWithFlywheel()
beam_with_flywheel.set_beam(iga_beam)
beam_with_flywheel.add_flywheel(flywheel)
beam_with_flywheel.set_attachment_point(iga_beam.get_attachment_point(0.5))  


sim_params = SimulationParameters()
sim_params.set_max_time(10.0)  
sim_params.set_gravity(0.0, -9.81, 0.0)  
sim_params.set_time_step(0.01)  


sim_params.add_rigid_body(rotary_motor)


sim_params.add_rigid_body(beam_with_flywheel)


viewer = IrrlichtViewer()
viewer.add_irrlicht_scene(iga_beam)
viewer.add_irrlicht_scene(beam_with_flywheel)


sim = Simulation(sim_params)
sim.add_rigid_body(rotary_motor)
sim.add_rigid_body(beam_with_flywheel)
sim.integrate(10000)  


viewer.start("Jeffcott_rotor_simulation.avi")