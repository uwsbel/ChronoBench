import chrono
import pychrono
import math
import numpy as np
import os
import sys
import time
import random


try:
    chrono = chrono.ChSystem()
    chrono.ChSystemNSC()
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    exit()


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)


chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)


class RigidTire:
    def __init__(self):
        self.position = chrono.ChVector3d(0, 0, 0)
        self.rotation = chrono.ChVector3d(0, 0, 0)

    def update(self, dt):
        
        self.rotation += 0.1 * math.sin(self.position * dt)
        self.position += self.rotation * dt

    def get_position(self):
        return self.position

    def get_rotation(self):
        return self.rotation


class RigidTerrain:
    def __init__(self):
        self.terrain_length = 100
        self.terrain_width = 100
        self.terrain_y = 0
        self.terrain_z = 0

    def get_position(self, x, y, z):
        return chrono.ChVector3d(x, y, z)

    def get_rotation(self, x, y, z):
        return chrono.ChVector3d(x, y, z)

    def update(self, dt):
        self.terrain_z += 0.01 * dt


class SimulationSystem:
    def __init__(self):
        self.body = chrono.ChBody()
        self.mesh = chrono.ChMesh()
        self.collision_system = chrono.ChCollisionSystem()
        self.terrain = chrono.ChTerrain()

    def add_element(self, element):
        self.body.AddElement(element)

    def set_collision_system(self, collision_system):
        self.collision_system = collision_system

    def set_terrain(self, terrain):
        self.terrain = terrain


class HMMWV:
    def __init__(self):
        self.position = chrono.ChVector3d(0, 0, 0)
        self.orientation = chrono.ChVector3d(0, 0, 0)
        self.rigid_tire = RigidTire()
        self.velocity = chrono.ChVector3d(0, 0, 0)
        self.acceleration = chrono.ChVector3d(0, 0, 0)

    def update(self, dt):
        self.position += self.velocity * dt
        self.orientation += self.acceleration * dt
        self.rigid_tire.update(dt)

        
        self.velocity -= self.rigid_tire.get_position() * 0.1 * math.sin(self.orientation * dt)

        
        self.orientation = self.orientation - self.velocity * 0.1 * math.sin(self.orientation * dt)

        return self.position, self.orientation, self.velocity, self.acceleration


class Simulation:
    def __init__(self):
        self.system = SimulationSystem()
        self.vehicle = HMMWV()
        self.terrain = RigidTerrain()

    def run(self, dt):
        self.system.Add(self.vehicle)
        self.system.Add(self.terrain)

        
        position, orientation, velocity, acceleration = self.vehicle.update(dt)

        
        print(f"Time: {time.time()}, Position: {position}, Orientation: {orientation}, Velocity: {velocity}, Acceleration: {acceleration}")

        return position, orientation, velocity, acceleration


if __name__ == "__main__":
    try:
        dt = chrono.Duration(0.01)  
        position, orientation, velocity, acceleration = self.simulation.run(dt)
        print("Simulation Complete.")
    except Exception as e:
        print(f"An error occurred: {e}")