import pychrono
import pychrono.core as pc
import pychrono.visuals as v
import pychrono.physics as pcf
import pychrono.utils as u
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm



truck_model = pc.VehicleModel(
    model_id="truck_model",
    max_speed=10,
    acceleration=0.5,
    deceleration=0.5,
    max_steering_angle=30,
    max_throttle=0.8,
    max_brake=0.3,
    mass=1000.0,
    
)


tmeasy_model = pc.TMEASY(
    model_id="tmeasy_model",
    tire_radius=0.4,
    tire_width=0.6,
    tire_pressure=0.8,
    max_speed=10,
    
)


terrain = pc.Terrain(
    name="mountain_terrain",
    heightmap=u.HeightMap(
        heightmap=np.random.rand(100, 100),
        heightmap_scale=1.0,
        
    ),
    
    slope_range=np.random.rand(100, 100),
    
)


simulation_time = 60  
time_step = 0.1 
simulation_start_time = u.Time(year=2024, month=10, day=26) 



simulation = pc.Simulation(
    time_step=time_step,
    simulation_start_time=simulation_start_time,
    time_limit=simulation_time,
    vehicle=truck_model,
    tmeasy=tmeasy_model,
    terrain=terrain,
    
)



fig = v.figure(figsize=(8, 6))
fig.set_window_title("MAN 10t Truck Simulation")
fig.set_ările(range(1, 1 + simulation.time_limit))


plt = plt.subplots()
plt.imshow(terrain.heightmap, cmap=cm.jet, interpolation='nearest')
plt.title("Terrain")
plt.colorbar(label="Height")
plt.show()


camera = v.Camera(
    width=800,
    height=600,
    
)
camera.set_position(0, 0, 5)  
camera.set_rotation(0, 90, 0)  
camera.set_focal_length(1000)  


steering_control = v.SteeringControl(
    
    max_steering_angle=30,
    min_steering_angle=0,
    max_steering_speed=0.5,
    min_steering_speed=0
)

throttle_control = v.ThrottleControl(
    max_throttle=0.8,
    min_throttle=0.2,
    max_throttle_speed=0.7,
    min_throttle_speed=0.3
)

brake_control = v.BrakeControl(
    max_brake=0.3,
    min_brake=0.1,
    max_brake_speed=0.6,
    min_brake_speed=0.2
)


for i in range(simulation.time_limit):
    
    truck_position = truck_model.position
    truck_orientation = truck_model.orientation

    
    steering_control.apply_control(truck_position, truck_orientation)

    
    throttle_control.apply_control(truck_position, truck_orientation)

    
    brake_control.apply_control(truck_position, truck_orientation)

    
    terrain.update(
        heightmap=u.HeightMap(
            heightmap=u.HeightMap(
                heightmap_scale=1.0,
                
            ),
            
        ),
        
    )

    
    v.render(fig, camera, truck_position, truck_orientation, terrain)

    
    print(f"Time: {i * time_step:.2f} seconds")


simulation.close()
plt.close()