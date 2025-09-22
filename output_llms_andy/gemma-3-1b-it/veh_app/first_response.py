import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import System, SensorManager

class GatorSimulation:
    def __init__(self, terrain_width=10, terrain_height=5, vehicle_mass=100, initial_position=(0, 0, 0),
                 initial_velocity=0, camera_resolution=64, sensor_manager=None):
        """
        Initializes the Gator simulation.

        Args:
            terrain_width (float): Width of the terrain.
            terrain_height (float): Height of the terrain.
            vehicle_mass (float): Mass of the Gator vehicle.
            initial_position (tuple): Initial position of the vehicle (x, y, z).
            initial_velocity (float): Initial velocity of the vehicle.
            camera_resolution (int): Resolution of the camera.
            sensor_manager (pychrono.core.SensorManager):  Optional sensor manager.
        """
        self.system = System()
        self.terrain = np.zeros((terrain_width, terrain_height), dtype=np.float32)
        self.vehicle = np.zeros((vehicle_mass, 3), dtype=np.float32)
        self.sensor_manager = sensor_manager if sensor_manager else None
        self.timestep = 0.0
        self.simulation_mode = False
        self.running = False
        self.time = 0.0

        # Initialize terrain
        self.terrain[0, :] = initial_position
        self.terrain[1, :] = initial_position
        self.terrain[2, :] = initial_position

        # Initialize vehicle
        self.vehicle[0] = initial_position
        self.vehicle[1] = initial_position
        self.vehicle[2] = initial_position

        self.sensor_manager = sensor_manager if sensor_manager else None

    def update_terrain(self):
        """Updates the terrain based on vehicle movement."""
        self.terrain = np.zeros((self.terrain_width, self.terrain_height), dtype=np.float32)
        for i in range(self.terrain_width):
            for j in range(self.terrain_height):
                if self.vehicle[0, 0] > 0 and self.vehicle[0, 0] < self.terrain_width and self.vehicle[1, 0] > 0 and self.vehicle[1, 0] < self.terrain_width:
                    self.terrain[i, j] = 1.0  # Simulate ground movement
        self.timestep += 1

    def update_vehicle(self):
        """Updates the vehicle's position and velocity."""
        self.vehicle[0] += self.vehicle[1] * 0.1
        self.vehicle[1] += self.vehicle[2] * 0.1
        self.vehicle[2] += self.vehicle[0] * 0.1

    def update_sensor_manager(self):
        """Updates the sensor manager with the current state."""
        if self.sensor_manager:
            self.sensor_manager.update(self.vehicle)

    def run_simulation(self):
        """Runs the simulation loop."""
        self.simulation_mode = True
        self.running = True
        while self.running:
            self.update_terrain()
            self.update_vehicle()
            self.update_sensor_manager()
            self.timestep += 1
            if self.timestep % 10 == 0:
                print(f"Time: {self.timestep}")
                self.time = self.timestep
                self.time = self.timestep
                plt.pause(1)  # Pause for 1 second to visualize

        self.simulation_mode = False
        print("Simulation finished.")


if __name__ == '__main__':
    # Example Usage
    simulation = GatorSimulation(terrain_width=15, terrain_height=8, vehicle_mass=50, initial_position=(2, 3, 0),
                                initial_velocity=2, camera_resolution=80, sensor_manager=None)
    simulation.run_simulation()