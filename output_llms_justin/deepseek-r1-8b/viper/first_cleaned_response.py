import chrono
from chrono.objects import *
from chrono.visualization import *
import numpy as np


sim = chrono.Simulation()
sim.set_gravity(True)
sim.add_ground_body("ground", mass=1000, is_static=True, size=(50, 50, 50))
sim.set_time_step(1.0/60)


rover = RigidBody("rover", mass=100, inertia=np.array([10, 10, 10]))
rover.set_shape("cube", dimensions=[1, 1.5, 2.5])
rover.set_position([0, 0, 0])
rover.set_orientation(chrono.PQ(1, 0, 0))
sim.add_object(rover)


driver = RigidBody("driver", mass=1, inertia=np.array([0.1, 0.1, 0.1]))
driver.set_position([1, 0.5, 0])
driver.set_shape("point")
sim.add_object(driver)


viz = Visualization(sim)
viz.set_camera("perspective", position=[5, 5, 5], look_at=[0, 0, 0])
viz.add_light("directional", position=[10, 10, 10], color=(1, 1, 1))
viz.add_light("ambient", position=[10, 10, 10], color=(1, 1, 1), intensity=0.5)


t = 0.0
t_total = 100.0
time_step = 1.0/60

while t <= t_total:
    
    steering_angle = np.sin(t) * 0.5  
    driver.set_position([np.sin(steering_angle), 0.5, 0])
    
    
    viz.render()
    sim.update()
    
    t += time_step
    
    time.sleep(1e-5)