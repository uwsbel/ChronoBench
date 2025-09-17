import pychrono as pyc
import numpy as np
import OpenCV as cv
import time


pyc.init()
cv.init()


physics_server = pyc.PyPhysicsServer()
visualization_server = pyc.VisualizationServer()


vehicle_body = pyc.RigidBody("Vehicle Body", [0, 0, 0], [0, 0, 0], [1, 0, 0], 0, 0, 0, 0, 0, 0, 0)


wheel_radius = 0.4
wheel_stiffness = 100000
wheel_damping = 1000


front_left_wheel = pyc.Wheel("Front Left Wheel", wheel_radius, wheel_stiffness, wheel_damping, [0.3, 0.5, 0.0], [0, 0, 0], [1, 0, 0])
front_right_wheel = pyc.Wheel("Front Right Wheel", wheel_radius, wheel_stiffness, wheel_damping, [0.3, 0.5, 0.0], [0, 0, 0], [1, 0, 0])
vehicle_body.add_component(front_left_wheel, 0.3, 0, 0)
vehicle_body.add_component(front_right_wheel, 0.3, 0, 0)


rear_left_wheel = pyc.Wheel("Rear Left Wheel", wheel_radius, wheel_stiffness, wheel_damping, [0.7, 0.5, 0.0], [0, 0, 0], [1, 0, 0])
rear_right_wheel = pyc.Wheel("Rear Right Wheel", wheel_radius, wheel_stiffness, wheel_damping, [0.7, 0.5, 0.0], [0, 0, 0], [1, 0, 0])
vehicle_body.add_component(rear_left_wheel, -0.3, 0, 0)
vehicle_body.add_component(rear_right_wheel, -0.3, 0, 0)


vehicle_pose = pyc.RigidPose([10.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
vehicle_body.set_pose(vehicle_pose)


terrain = pyc.SCMTerrain("Deformable Terrain", "data/soil_parameters.txt")
terrain.set_size(100, 100)
terrain.set_patch_size(0.5)
terrain.set_max_patch_speed(5.0)
terrain.enable_sinkage_visualization()
terrain.addVisualization(cv.createVideoWriter("output.mp4", 50))


driver = pyc.Driver("Driver", [0, 0, 0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
driver.add_control("steering", 0.0, 1.0, 2.0)
driver.add_control("throttle", 0.0, 1.0, 0.1)
driver.add_control("brake", 0.0, 0.0, 1.0)


terrain.add_object(vehicle_body, vehicle_pose)


physics_server.add_body(vehicle_body)
physics_server.add_terrain(terrain)
physics_server.set_gravity([0.0, 0.0, -9.81])


visualization_server.add_terrain(terrain)
visualization_server.add_body(vehicle_body, vehicle_pose)
visualization_server.add_mesh("Vehicle Body", "data/vehicle_mesh.obj", 0.5, 0.5, 0.5)
visualization_server.add_mesh("Front Left Wheel", "data/wheel_mesh.obj", wheel_radius, wheel_radius, wheel_radius)
visualization_server.add_mesh("Front Right Wheel", "data/wheel_mesh.obj", wheel_radius, wheel_radius, wheel_radius)
visualization_server.add_mesh("Rear Left Wheel", "data/wheel_mesh.obj", wheel_radius, wheel_radius, wheel_radius)
visualization_server.add_mesh("Rear Right Wheel", "data/wheel_mesh.obj", wheel_radius, wheel_radius, wheel_radius)


physics_server.start()
visualization_server.start()


steering = 0.0
throttle = 0.0
brake = 0.0

while True:
    
    steering = driver.get_control("steering")
    throttle = driver.get_control("throttle")
    brake = driver.get_control("brake")
    
    
    vehicle_body.set_control([steering, throttle, brake])
    
    
    visualization_server.render()
    
    
    time.sleep(1/50)