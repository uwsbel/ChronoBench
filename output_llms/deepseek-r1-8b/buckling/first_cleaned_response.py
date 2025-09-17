import pychrono as pychrono
import pychrono.ophysics as physics
import pychrono visualize as visualize
import pychrono.gui as gui
import pychrono.fea as fea
import pychrono.mechanisms as mechanisms
import pychrono.dynamics as dynamics
import pychrono.math as math
import pychrono.utils as utils


pychrono.init()


engine = pychrono.Engine()
engine.set_solver(dynamics.SolverType.ODE, dynamics.ODEType.RK4, dynamics.RK4Order())
engine.set_timestep(1e-5)  


renderer = visualize.IrrlichtRenderer()
renderer.set_lighting(True)
renderer.set_camera(position=(5, 5, 5), look_at=(0, 0, 0))
renderer.set_render_window_size(800, 600)



beam_length = 4.0  
beam_diameter = 0.1  
young_modulus = 210e9  
poisson_ratio = 0.2  
density = 8000  


beam1 = mechanisms.Rod(
    name="beam1",
    material=physics.Material(young_modulus=young_modulus, poisson_ratio=poisson_ratio, density=density),
    cross_section=physics.Rectangle(diameter=beam_diameter, thickness=0.1),
    mass_per_unit_length=0.1  
)
beam2 = mechanisms.Rod(
    name="beam2",
    material=physics.Material(young_modulus=young_modulus, poisson_ratio=poisson_ratio, density=density),
    cross_section=physics.Rectangle(diameter=beam_diameter, thickness=0.1),
    mass_per_unit_length=0.1
)


fixed = mechanisms.Fixed()
fixed.add(beam1)
fixed.add(beam2)


pin = mechanisms.Pin()
pin.add(beam1, "end1")
pin.add(beam2, "end2")


def motor_force(t):
    return 1000.0 * (1.0 - (t/10.0))  

motor = mechanisms.Motor()
motor.add(beam1, "end1", force_function=motor_force)


engine.add_object(motor)



beam1.add_visual(visualize.RodVisual(0, 0, 0, beam_diameter/2, 0.1, color=(1, 0, 0)))
beam2.add_visual(visualize.RodVisual(beam_length, 0, 0, beam_diameter/2, 0.1, color=(1, 0, 0)))


fixed.add_visual(visualize.FixedVisual(position=(0, 0, 0)))
pin.add_visual(visualize.PinVisual())


motor.add_visual(visualize.MotorVisual(position=beam1.position + (0, 0, 0)))


engine.run()