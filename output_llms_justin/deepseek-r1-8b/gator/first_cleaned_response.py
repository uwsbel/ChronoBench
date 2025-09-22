import pychrono as pychrono
import pychrono.vehicles as vehicles
import pychrono.render as render
import pygame
import os
import numpy as np


pychrono.init()
sim = pychrono.Simulation()


gator = vehicles.Vehicle(sim, "Gator")
gator.set_name("Gator")
gator.set_contact_method(vehicles.ContactMethod.FULL)
gator.set_tire_model("TMEASY")
gator.set_location(pychrono.Vec3(0, 0, 0))
gator.set_orientation(pychrono.Vec3(0, 0, 1))
gator.set_mass(1000)
gator.set_inertia_matrix(
    np.array([
        [1000, 0, 0],
        [0, 1000, 0],
        [0, 0, 1000]
    ])
)


terrain = pychrono.objects.RigidTerrain(sim, "RigidTerrain")
terrain.set_size(pychrono.Vec3(100, 100, 0))
terrain.set_texture(os.path.join("textures", "terrain.png"))
terrain.set_height_function(
    lambda x, y: 0.5 * (1 - abs(2 * x / 100)) * (1 - abs(2 * y / 100))
)
terrain.set_static(True)


chassis = pychrono.objects.SimpleObject(sim, "Chassis")
chassis.set_mass(500)
gator.add_component(chassis)
gator.set_chassis_component(chassis)


wheel1 = pychrono.objects.SimpleObject(sim, "Wheel")
wheel1.set_location(pychrono.Vec3(-20, -30, 0))
wheel1.set_rotation(pychrono.Vec3(0, 1, 0))
gator.add_component(wheel1)
gator.set_wheel_component(0, wheel1)

wheel2 = pychrono.objects.SimpleObject(sim, "Wheel")
wheel2.set_location(pychrono.Vec3(20, -30, 0))
wheel2.set_rotation(pychrono.Vec3(0, 1, 0))
gator.add_component(wheel2)
gator.set_wheel_component(1, wheel2)


suspension = pychrono.objects.SimpleObject(sim, "Suspension")
suspension.set_location(pychrono.Vec3(0, 0, 0))
gator.add_component(suspension)
gator.set_suspension_component(0, suspension)


renderer = render.IrrlichtRenderer(sim)
window = renderer.create_window(800, 600, "PyChrono Gator")
renderer.set_window_size(800, 600)
renderer.enable_key_event Handling
renderer.enable_mouse_event Handling


class InputHandler:
    def __init__(self):
        self.steering = 0
        self.throttle = 0
        self.brake = 0

    def handle_event(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                self.steering = 1
            elif event.key == pygame.K_RIGHT:
                self.steering = -1
            elif event.key == pygame.K_UP:
                self.throttle = 1
            elif event.key == pygame.K_DOWN:
                self.brake = 1

    def get_input(self):
        return self.steering, self.throttle, self.brake

input_handler = InputHandler()


clock = pygame.time.Clock()
running = True

while running:
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        input_handler.handle_event(event)

    
    steering, throttle, brake = input_handler.get_input()

    
    sim.step(pychrono.Simulation.SIMSTEP_TYPE.PHYSICS_ONLY)
    renderer.update()

    
    renderer.draw_scene()

    
    clock.tick(50)

pygame.quit()