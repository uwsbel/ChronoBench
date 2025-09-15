import pychrono as pychrono
import pychrono.render as pyrender
import pychrono.input as pyinput
import numpy as np


pychrono.init()


simulation = pychrono.Simulation("PyChrono", window_size=(800, 600))
physics = simulation.get_physics()
physics.set_physics_engine("Bullet", 1e6)
scene = simulation.get_scene()


renderer = pyrender.IrrlichtRenderer()
renderer.set_background_color(pyrender.Color(0, 0, 0))
renderer.set_fbo_size(800, 600)
simulation.get_window().set_renderer(renderer)


car_body = pychrono.RigidBody("car_body")
car_body.set_mass(1000)
car_body.set_size(4, 2, 2)
car_body.set_position(pychrono.Vector3(0, 0, 0))
car_body.set_color(pychrono.Color(0, 0, 0))  


wheels = []
for i in range(4):
    wheel = pychrono.RigidBody("wheel")
    wheel.set_mass(10)
    wheel.set_size(0.5, 0.5, 0.5)
    wheel.set_position(pychrono.Vector3(
        -1.5, -1, 0.5  
    ) if i < 2 else 1.5, -1, 0.5)
    wheel.set_color(pychrono.Color(0, 0, 0))
    wheels.append(wheel)


car_body.add_child(wheels[0], pychrono.Vector3(1.5, 0, 0))
car_body.add_child(wheels[1], pychrono.Vector3(-1.5, 0, 0))
car_body.add_child(wheels[2], pychrono.Vector3(1.5, 0, 0))
car_body.add_child(wheels[3], pychrono.Vector3(-1.5, 0, 0))


terrain = pychrono.RigidBody("terrain")
terrain.set_mass(0)
terrain.set_size(100, 100, 5)
terrain.set_position(pychrono.Vector3(0, -5, 0))
terrain.set_color(pychrono.Color(1, 1, 1))  


terrain.add_texture("grass", "grass.png")
terrain.add_texture("logo", "bmw.png")
terrain.apply_textures()


car_body.add_constraint(pychrono.HingeConstraint(
    pychrono.Vector3(1, 0, 0),  
    pychrono.Vector3(0, 0, 0),  
    pychrono.Vector3(0, 1, 0),  
    pychrono.Vector3(0, 0, 0)  
))


steering = pyinput.ActionNode("steering", pyinput.InputType.Slider, range(-1, 1))
throttle = pyinput.ActionNode("throttle", pyinput.InputType.Slider, range(-1, 1))
braking = pyinput.ActionNode("braking", pyinput.InputType.Slider, range(-1, 1))


physics.setVehicleFriction(1, 1, 1)
physics.setVehicleDamping(0.5, 0.5, 0.5)
physics.setVehicleRestitution(0.3)


tire = pychrono.RigidBody("tire")
tire.set_mass(10)
tire.set_size(0.1, 0.1, 0.1)
tire.set_position(pychrono.Vector3(0, 0, 0))
tire.set_model("TMEASY")
tire.add_scalar(0, "damping", 0.5)
tire.add_scalar(0, "num_segments", 50)
tire.add_scalar(0, "hardness", 0.8)


wheels[0].add_child(tire, pychrono.Vector3(0, 0, 0))
wheels[1].add_child(tire, pychrono.Vector3(0, 0, 0))
wheels[2].add_child(tire, pychrono.Vector3(0, 0, 0))
wheels[3].add_child(tire, pychrono.Vector3(0, 0, 0))


camera = scene.add_camera("chase_camera", pychrono.RendererCameraType.Perspective)
camera.set_position(pychrono.Vector3(0, 5, 10))
camera.set.look_at(pychrono.Vector3(0, 0, 0))
renderer.set_camera(camera)


light = pyrender.DirectionalLight(
    pyrender.Color(1, 1, 1),
    pyrender.Vector3(10, 10, 10)
)
renderer.add_light(light)


skybox = pyrender.CubeTexture("sky", "skybox.png")
skybox.set_size(100, 100, 100)
skybox.set_position(pychrono.Vector3(0, 0, 0))
renderer.add_sphere(skybox)


simulation.get_window().set_key_callback(pyinput.KeyboardEvent, "KEY_PRESSED", "ToggleBrake")
simulation.get_window().set_mouse_callback(pyinput.MouseEvent, "MOUSE_DRAG", "CameraPan")
simulation.get_window().set_mouse_wheel_callback(pyinput.MouseWheelEvent, "CameraZoom")


car_body.add_collision_detector()
terrain.add_collision_detector()


physics.setVehicleSteerRatio(0.5)
physics.setVehicleThrottleRatio(0.5)
physics.setVehicleBrakeRatio(0.5)


simulation.start()


def handle_input():
    global steering, throttle, braking
    speed = physics.getVehicleLinearVelocity()
    angle = physics.getVehicleAngularVelocity()
    
    
    steering.set_value(steering.get_value() * speed[0] * 0.5)
    
    
    if speed[0] < 0.5:
        throttle.set_value(throttle.get_value() * (0.5 - speed[0]))
    
    
    if speed[0] > 0.5:
        braking.set_value(braking.get_value() * (speed[0] - 0.5))
    
    
    steering.update()
    throttle.update()
    braking.update()


simulation.run_loop():
    handle_input()
    physics.update()
    renderer.render()