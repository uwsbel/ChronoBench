import pychrono
import pychrono.objects as o
import pychrono.visuals as v
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


class CrankAndSlider(o.Object):
    def __init__(self, mass=1.0, length=1.0, radius=0.5, angle=0.0):
        super().__init__()
        self.mass = mass
        self.length = length
        self.radius = radius
        self.angle = angle
        self.rotation_speed = 0.0  
        self.current_angle = 0.0
        self.dynamic_force = 0.0
        self.is_active = False

    def update(self, dt):
        self.current_angle += self.angle * dt
        self.dynamic_force = self.mass * self.length * self.angle * 3.14159 * np.sin(self.current_angle)
        self.rotation_speed = 60 * np.sin(self.current_angle)  
        self.is_active = True

    def get_position(self):
        return self.rotation_speed * self.length

    def set_position(self, position):
        self.rotation_speed = position

    def get_force(self):
        return self.dynamic_force

    def set_force(self, force):
        self.dynamic_force = force


class Motor(o.Object):
    def __init__(self, speed=50.0):
        super().__init__()
        self.speed = speed

    def update(self, dt):
        self.rotation_speed = self.speed * dt


class Floor(o.Object):
    def __init__(self, length=1.0, width=0.5, height=0.2):
        super().__init__()
        self.length = length
        self.width = width
        self.height = height

    def get_position(self):
        return self.length * 0.5  


class Crankshaft(o.Object):
    def __init__(self, length=1.0, radius=0.5):
        super().__init__()
        self.length = length
        self.radius = radius

    def get_position(self):
        return self.length * 0.5


class ConnectingRod(o.Object):
    def __init__(self, length=1.0):
        super().__init__()
        self.length = length

    def get_position(self):
        return self.length


class Piston(o.Object):
    def __init__(self, length=1.0, radius=0.5):
        super().__init__()
        self.length = length
        self.radius = radius

    def get_position(self):
        return self.length * 0.5


system = CrankAndSlider()


floor = Floor()
system.add_object(floor)


crankshaft = Crankshaft()
system.add_object(crankshaft)


connecting_rod = ConnectingRod()
system.add_object(connecting_rod)


piston = Piston()
system.add_object(piston)


system.set_position(floor.get_position())
crankshaft.set_position(crankshaft.get_position())
connecting_rod.set_position(connecting_rod.get_position())
piston.set_position(piston.get_position())


system.set_force(motor.get_force())


v.Simulation(system,  
            title="Simple Crank-Slider Simulation",
            camera_settings={"camera_position": (0, 0, 0)},
            
            
            
            
            
            
            
            )


v.wait_for_simulation()