import pychrono as pychrono
import pychrono.trajectory
import pychrono.visualization
import pychrono.dynamics
import pychrono.guitools
import pychrono.objects


pychrono.init()


world = pychrono.World(pychrono.PyOde2())
world.set_gravity([0, 0, -9.81])


ground = pychrono.objects.Plane(world, "ground", pychrono.Vector3(0, 0, 0), pychrono.Vector3(10, 10, 0))
ground.set_mass(0)
ground.set_friction(1.0)
ground.set_damping(0.5)
ground.set_collision_response(1.0)


rover_body = pychrono.objects.DynamicBody(world, "rover", pychrono.Vector3(0, 0.5, 0), pychrono.Vector3(1, 1, 1))
rover_body.set_mass(200)
rover_body.set_inertia(pychrono.Vector3(1, 1, 1))
rover_body.set_color(pychrono.Vector4(1, 0.5, 0, 1))


wheel1 = pychrono.objects.Wheel(rover_body, "wheel1", pychrono.Vector3(0.5, 0, 0), pychrono.Vector3(0, 0, 1))
wheel2 = pychrono.objects.Wheel(rover_body, "wheel2", pychrono.Vector3(-0.5, 0, 0), pychrono.Vector3(0, 0, 1))
wheel3 = pychrono.objects.Wheel(rover_body, "wheel3", pychrono.Vector3(0, 0.5, 0), pychrono.Vector3(1, 0, 0))
wheel4 = pychrono.objects.Wheel(rover_body, "wheel4", pychrono.Vector3(0, -0.5, 0), pychrono.Vector3(1, 0, 0))


motor_control = pychrono.dynamics.MotorControl()
rover_body.set_motor_control(motor_control, pychrono.MotorControlType.Torque, 4)


renderer = pychrono.visualization.IrrlichtRenderer(world, pychrono.IrrlichtRenderer.PerspectiveCamera(45, 1.0, 1000))
renderer.set_camera_position(pychrono.Vector3(5, 5, 5))
renderer.set_antialiasing(8)
renderer.set_shadows(1)
renderer.set_light(1, pychrono.Vector3(10, 10, 10), pychrono.Vector3(0, 0, 0))
renderer.add_texture("skybox", "skybox texture")
renderer.add_texture("rover", "curiosity texture")


world.add_object(ground, pychrono.objects.Link(pychrono.Vector3(0, 0, 0), pychrono.Vector3(0, 0, 1)))


simulation = pychrono.Simulation(world, pychrono.Simulation.SIMULATION_TIME_MS * 1000)
simulation.set_real_time(1)


view = pychrono.guitools.Viewer(renderer, pychrono.guitools.PanControls())
view.add_camera_controls()
view.add_light_controls()


while True:
    simulation.update()
    renderer.update()
    view.update()
    
    
    if view.has_camera_controls():
        view.camera_control.update()
    if view.has_light_controls():
        view.light_control.update()
    
    
    motor_control.set_torque_output([0, 0, 0, 0])
    
    if pychrono.input().get_key('q'):
        motor_control.set_torque_output([1.0, 0, 0, 0])
    if pychrono.input().get_key('w'):
        motor_control.set_torque_output([-1.0, 0, 0, 0])
    
    renderer.draw()
    pychrono.sleep(1e-3)