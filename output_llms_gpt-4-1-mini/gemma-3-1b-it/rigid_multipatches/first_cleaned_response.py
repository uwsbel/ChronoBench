import pychrono
import pychrono.core as chrono
import pychrono.visuals as visual
import pychrono.physics as physics
import pychrono.utils as utils
import numpy as np
import matplotlib.pyplot as plt  




class HMMWVSimulation:
    def __init__(self, terrain_width=100, terrain_height=20, vehicle_width=20, vehicle_height=10,
                 engine_type="diesel", drivetrain_type="automatic",
                 vehicle_position=(0, 0, 0),
                 max_speed=5,
                 timestep=0.1):
        
        self.terrain_width = terrain_width
        self.terrain_height = terrain_height
        self.vehicle_width = vehicle_width
        self.vehicle_height = vehicle_height
        self.engine_type = engine_type
        self.drivetrain_type = drivetrain_type
        self.vehicle_position = vehicle_position
        self.max_speed = max_speed
        self.timestep = timestep

        self.terrain = np.zeros((terrain_width, terrain_height), dtype=np.float32)
        self.vehicle = np.zeros((vehicle_width, vehicle_height), dtype=np.float32)

        self.terrain[0, :] = 0.0  
        self.vehicle[0, :] = 0.0

        self.terrain[0, 0] = 0.0  
        self.vehicle[0, 0] = 0.0

        self.terrain[0, 1] = 0.0  
        self.vehicle[0, 1] = 0.0

        self.terrain[0, 2] = 0.0  
        self.vehicle[0, 2] = 0.0

        self.terrain[1, 0] = 0.0 
        self.vehicle[1, 0] = 0.0

        self.terrain[1, 1] = 0.0 
        self.vehicle[1, 1] = 0.0

        self.terrain[1, 2] = 0.0 
        self.vehicle[1, 2] = 0.0


    def update_terrain(self):
        
        for x in range(self.vehicle_width):
            for y in range(self.vehicle_height):
                self.terrain[x, y] += self.vehicle[x, y] * self.timestep

    def update_vehicle(self):
        
        self.vehicle[0, 0] += self.timestep * self.engine_type
        self.vehicle[0, 1] += self.timestep * self.engine_type
        self.vehicle[1, 0] += self.timestep * self.engine_type
        self.vehicle[1, 1] += self.timestep * self.engine_type


    def visualize_terrain(self):
        
        plt.imshow(self.terrain, cmap='gray', interpolation='nearest')
        plt.title("Terrain")
        plt.show()

    def visualize_vehicle(self):
        
        plt.imshow(self.vehicle, cmap='viridis', interpolation='nearest')
        plt.title("Vehicle")
        plt.show()




def simulate_hmmwv(simulation, terrain_width=100, terrain_height=20, vehicle_width=20, vehicle_height=10,
                  engine_type="diesel", drivetrain_type="automatic", vehicle_position=(0, 0, 0),
                  max_speed=5, timestep=0.1):
    
    simulation.update_terrain()
    simulation.update_vehicle()

    
    if simulation.vehicle[0, 0] > 10:  
        print("Collision detected!")
        simulation.update_terrain() 
        simulation.update_vehicle()


    print(f"Simulation time: {simulation.timestep}")




if __name__ == "__main__":
    simulation = HMMWVSimulation(terrain_width=100, terrain_height=20, vehicle_width=20,
                                 vehicle_height=10, engine_type="gasoline", drivetrain_type="automatic",
                                 vehicle_position=(50, 50, 0), max_speed=6, timestep=0.05)

    simulate_hmmwv(simulation, terrain_width=100, terrain_height=20, vehicle_width=20, vehicle_height=10,
                  engine_type="diesel", drivetrain_type="automatic", vehicle_position=(20, 20, 0),
                  max_speed=6, timestep=0.05)