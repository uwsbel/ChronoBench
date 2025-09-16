import pychrono
import pychrono.visuals as v
import numpy as np
import matplotlib.pyplot as plt
from pychrono.visuals import Frame
from pychrono.visuals import Color


num_steps = 1000
gear_ratio = 2.0  
fixed_truss_length = 1.0
rotating_bar_radius = 0.5
gear_motor_speed = 10.0  
truss_mass = 1.0
bar_mass = 1.0
gear_motor_weight = 0.1


simulation = pychrono.simulation.Simulation()


frame = Frame(
    name="epicyclic_gears",
    num_steps=num_steps,
    fixed_truss_length=fixed_truss_length,
    rotating_bar_radius=rotating_bar_radius,
    gear_motor_speed=gear_motor_speed,
    gear_ratio=gear_ratio,
    bar_mass=bar_mass,
    gear_motor_weight=gear_motor_weight
)


frame.add_object("fixed_truss",
                name="fixed_truss",
                type="frame",
                mass=truss_mass,
                position=(0, 0, 0),
                color=Color("gray"))


frame.add_object("rotating_bar",
                name="rotating_bar",
                type="frame",
                mass=bar_mass,
                position=(0, 0, rotating_bar_radius),
                color=Color("blue"))


frame.add_object("gear_1",
                name="gear_1",
                type="frame",
                mass=gear_motor_weight,
                position=(0, 0, 0),
                color=Color("red"))
frame.add_object("gear_2",
                name="gear_2",
                type="frame",
                mass=gear_motor_weight,
                position=(0, 0, 0),
                color=Color("green"))


simulation.set_physics(frame)
simulation.set_simulation_mode(pychrono.simulation.SimulationMode.REALTIME)


v.plot(simulation,
       "fixed_truss",
       color="gray",
       linewidth=1,
       label="Fixed Truss")
v.plot(simulation,
       "rotating_bar",
       color="blue",
       linewidth=1,
       label="Rotating Bar")
v.plot(simulation,
       "gear_1",
       color="red",
       linewidth=1,
       label="Gear 1")
v.plot(simulation,
       "gear_2",
       color="green",
       linewidth=1,
       label="Gear 2")

v.show(simulation)