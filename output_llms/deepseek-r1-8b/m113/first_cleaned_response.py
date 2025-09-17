import pychrono as pychrono
from pychrono import Vehicle, Driver, RigidBody, Physics
from pychrono.trajectory import Trajectory
from pychrono.utils import *
from pychrono.render import *
import time


pychrono.init()


physics = Physics()
physics.set_real_time(True)
physics.set_timestep(1.0/60)  


vehicle = Vehicle("M113", mass=3000,  
                  dimensions=[2.5, 2.5, 1.2],  
                  inertia=[[4.0, 0, 0], [0, 4.0, 0], [0, 0, 4.0]])  
vehicle.set_initial_position([0, 0, 0])
vehicle.set_initial_orientation(quat_from_euler([0, 0, 0, 1]))
vehicle.set_initial_velocity([0, 0, 0])


driver = Driver("Driver")
driver.attach_to(vehicle)


terrain = RigidBody("Terrain")
terrain.set_mass(0)
terrain.set_friction(0.5)  
terrain.set_restitution(0.3)  
terrain.create_plane(position=[0, 0, 0], size=[100, 100, 1])
terrain.set_ground(True)


renderer = create_renderer()
renderer.set_camera_distance(5, 5, 5)  
renderer.set_light_setup()
renderer.set_light_direction(10, 10, 10)  


trajectory = Trajectory()
trajectory.set_name("M113_Simulation")

while True:
    physics.update()
    time.sleep(1.0/60)  
    
    
    vehicle_state = vehicle.get_state()
    terrain_state = terrain.get_state()
    
    
    speed = vehicle_state.velocity().z
    yaw = vehicle_state.yaw
    
    
    keys = pygame.keys()
    speed = 0
    steering = 0
    
    if 'W' in keys:
        speed = 1
    elif 'S' in keys:
        speed = -1
    if 'A' in keys:
        steering = 0.5
    elif 'D' in keys:
        steering = -0.5
    
    
    driver.set_throttle(speed)
    driver.set_steering(steering)
    
    
    renderer.update()
    
    
    trajectory.add_point(vehicle_state.position)
    trajectory.add_vector(vehicle_state.velocity)
    
    
    print(f"Speed: {speed:.2f} m/s | Yaw: {yaw:.2f} rad")
    
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT and 'Q' in keys:
            pygame.quit()
            exit()