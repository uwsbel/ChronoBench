```python
import pychrono
import pychrono.core as pc
import pychrono.geometry as pg
import pychrono.physics as pcp
import pychrono.visuals as v


try:
    chrono = pc.chrono.Chronos()
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()








rotor_radius = 0.5  
rotor_mass = 100.0  
rotor_angle = 0.0  
rotor_velocity = 0.0  


flywheel_mass = 50.0  
flywheel_radius = 0.1  
flywheel_center = (0, 0) 


motor_mass = 20.0  
motor_velocity = 1.0  
motor_angle = 0.0 



beam_radius = 0.05  
beam_length = 1.0  
beam = pg.Beam(
    radius=beam_radius,
    length=beam_length,
    material=pc.Material('steel'),
    
)



beam.attach_object(flywheel, beam_center,  
                 beam_radius, beam_length,
                 flywheel_mass, flywheel_radius)



motor_x = 0.0
motor_y = 0.0
motor_z = beam_radius  
motor_angle = 0.0

motor = pg.Motor(
    x=motor_x,
    y=motor_y,
    z=motor_z,
    mass=motor_mass,
    velocity=motor_velocity,
    angle=motor_angle
)



rotor_x = 0.0
rotor_y = 0.0
rotor_z = 0.0
flywheel_x = flywheel_center[0]
flywheel_y = flywheel_center[1]
flywheel_z = flywheel_center[2]
motor_x = motor_x
motor_y = motor_y
motor_z = motor_z


try:
    
    chrono.set_simulation_parameters(
        time_step=0.01,  
        simulation_name="Jeffcott_Rotor_Simulation",
        gravity_acceleration=9.81,
        frame_rate=30
    )

    
    chrono.run(