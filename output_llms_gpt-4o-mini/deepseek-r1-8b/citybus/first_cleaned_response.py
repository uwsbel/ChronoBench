import PyChrono as pc
import PyIrrlicht as ir
import math
import time


pc.init()
ir.init()


simulation = pc.Simulation()
scene = simulation.scene


vehicle = pc.RigidVehicle2D()
vehicle.name = "CityBus"
vehicle.position = [0, 0, 0]
vehicle.orientation = 0
vehicle.tire_model = pc.Tire2DModel()  


mesh = pc.Mesh()
mesh.type = pc.MeshType.Mesh


mesh.vertices = [
    [-1, -1, 0],
    [1, -1, 0],
    [1, 1, 0],
    [-1, 1, 0],
    [0, 0, 1],
    [0, 0, -1]
]
mesh.material = pc.Material()
mesh.material.diffuse = [1, 1, 1]
scene.add_object(mesh)


wheel_positions = [[1, 0], [-1, 0], [0, 1], [0, -1]]
tires = []
for pos in wheel_positions:
    tire = pc.Tire2D()
    tire.position = pos
    tire.tire_model = pc.Tire2DModel()
    tires.append(tire)
for tire in tires:
    scene.add_object(tire)


terrain = pc.RigidTerrain()
terrain.type = pc.TerrainType.Plane
terrain.position = [0, 0, 0]
terrain.rotation = 0
terrain.material = pc.Material()
terrain.material.diffuse = [0.8, 0.2, 0.2]  
scene.add_object(terrain)


camera = pc.Camera()
camera.position = [0, 5, 5]
camera.look_at = [0, 0, 0]
camera fov = 45
scene.add_object(camera)


def handle_input():
    speed = 0
    steering = 0
    if pc.Input.is_pressed(pc.Input.Key.Key_D):
        speed = 0.1
    elif pc.Input.is_pressed(pc.Input.Key.Key_A):
        speed = -0.1
    if pc.Input.is_pressed(pc.Input.Key.Key_W):
        steering = 0.1
    elif pc.Input.is_pressed(pc.Input.Key.Key_S):
        steering = -0.1
    return speed, steering


target_fps = 50
step = 1.0 / target_fps
simulation.step = step

while True:
    
    speed, steering = handle_input()
    vehicle.velocity = [speed, 0]
    vehicle.angle = -steering * 0.1  

    
    camera.position = [camera.position.x + speed * 10, camera.position.y]
    camera.look_at = [0, 0, 0]

    
    scene.clear()
    scene.add_light()
    scene.add_object(terrain)
    scene.add_object(vehicle)
    scene.add_object(camera)
    ir.update()

    
    time.sleep(step)


pc.close()