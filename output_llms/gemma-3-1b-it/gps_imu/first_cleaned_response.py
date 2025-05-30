import pychrono
import time
import random
import math
import irlayout as ir


simulation_time = 60  
dt = 0.1  
gravity = 9.81  
terrain_heightmap = {
    "forest": 0.1,
    "mountain": 0.5,
    "desert": 0.3,
    "urban": 0.2
}
vehicle_mass = 1000  
driver_speed = 2.0  
driver_acceleration = 0.1 
sensor_range = 10 
sensor_accuracy = 0.01 


simulation = pychrono.Simulation()
simulation.set_time(0, simulation_time)  
simulation.set_physics(True) 



vehicle = {
    "chassis": {
        "x": 0,
        "y": 0,
        "z": 0,
        "mass": vehicle_mass,
        "position": (0, 0, 0)
    },
    "orientation": {
        "x": 0,
        "y": 0,
        "z": 0
    }
}


terrain = {
    "forest": {
        "heightmap": terrain_heightmap,
        "heightmap_scale": 1.0
    },
    "mountain": {
        "heightmap": terrain_heightmap,
        "heightmap_scale": 0.8
    },
    "desert": {
        "heightmap": terrain_heightmap,
        "heightmap_scale": 0.5
    },
    "urban": {
        "heightmap": terrain_heightmap,
        "heightmap_scale": 0.2
    }
}


driver = {
    "x": 0,
    "y": 0,
    "z": 0,
    "speed": driver_speed,
    "acceleration": driver_acceleration
}


imu = {
    "x": 0,
    "y": 0,
    "z": 0
}
gps = {
    "x": 0,
    "y": 0,
    "z": 0
}


simulation.add_system("vehicle")
simulation.add_system("terrain")
simulation.add_system("driver")
simulation.add_system("imu")
simulation.add_system("gps")


for step in range(simulation.time):
    
    imu.update()
    gps.update()
    
    
    vehicle["orientation"].x += vehicle_mass * dt * math.cos(math.radians(vehicle["orientation"].z))
    vehicle["orientation"].y += vehicle_mass * dt * math.sin(math.radians(vehicle["orientation"].z))
    vehicle["position"] = (vehicle["x"], vehicle["y"], vehicle["z"])

    
    terrain["heightmap"][vehicle["position"][0]] += 0.01  
    terrain["heightmap"][vehicle["position"][1]] += 0.01
    terrain["heightmap"][vehicle["position"][2]] += 0.01

    
    if vehicle["orientation"].x > 0:
        driver["acceleration"] = -0.5 * dt * math.sin(math.radians(vehicle["orientation"].z))
    elif vehicle["orientation"].x < 0:
        driver["acceleration"] = 0.5 * dt * math.sin(math.radians(vehicle["orientation"].z))
    else:
        driver["acceleration"] = 0.0

    
    print(f"Step: {step}, Time: {step * dt:.2f} s")
    print(f"Vehicle Position: {vehicle['position']}")
    print(f"Driver Acceleration: {driver['acceleration']}")

    
    simulation.update()


simulation.stop()
print("Simulation finished.")