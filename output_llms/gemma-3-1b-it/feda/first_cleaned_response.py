import pychrono
import pychrono.core as chrono
import pychrono.objects as obj
import pychrono.scenes as scenes
import pychrono.utils as utils
import pychrono.graphics as graphics
import pychrono.physics as physics
import pychrono.ui as ui
import random


def create_terrain(width, height):
    
    terrain = obj.Terrain(width, height)
    terrain.set_color('gray')  
    return terrain


def create_vehicle(location, orientation, contact_method, tire_model, speed=1.0):
    
    vehicle = obj.Vehicle(
        location=location,
        orientation=orientation,
        contact_method=contact_method,
        tire_model=tire_model,
        speed=speed,
        mass=10.0 
    )
    return vehicle


def set_camera_position(camera_position, camera_rotation):
    
    camera = ui.Camera(position=camera_position, rotation=camera_rotation)
    return camera


def create_rigid_terrain(width, height, texture_name="grass"):
    
    terrain = create_terrain(width, height)
    terrain.set_texture(texture_name)
    return terrain



def initialize_simulation(vehicle, camera, terrain):
    
    simulation = chrono.Simulation(
        name="FEDA_Simulation",
        initial_time=0.0,
        max_time=10.0,  
        frame_rate=50,
        physics_system=physics.RigidTerrain,
        camera=set_camera_position(camera, camera_rotation),
        vehicle=vehicle,
        terrain=terrain
    )
    return simulation



def main():
    
    simulation = initialize_simulation(vehicle, camera, terrain)

    
    simulation.set_physics_system(physics.RigidTerrain)
    simulation.set_camera(camera)
    simulation.set_vehicle(vehicle)
    simulation.set_terrain(terrain)

    
    simulation.run()

    
    if simulation.is_running():
        simulation.render()  
        print("Simulation rendered.")
    else:
        print("Simulation is not running.")

if __name__ == "__main__":
    main()