import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import Chrono
from pychrono.core.sensor import Sensor
from pychrono.core.simulation import Simulation
from pychrono.core.environment import Environment
from pychrono.core.observer import Observer
from pychrono.core.physics import Physics
from pychrono.core.event import Event


env = Environment()



ground_body = {
    'type': 'sphere',
    'radius': 1.0,
    'mass': 10.0,  
    'position': (0.0, 0.0, 0.0),
    'velocity': (0.0, 0.0, 0.0) 
}


sensors = [
    Sensor('camera', 'camera_sensor',  
              'resolution': 640,  
              'frame_rate': 30,
              'noise_level': 0.01) 
]


ground_body['velocity'] = (0.0, 0.0, 0.0) 


sensor_manager = SensorManager()
sensor_manager.add_sensors(sensors)


chrono = Chrono(env, sensor_manager)


def run_simulation():
    
    try:
        
        chrono.run()
    except Exception as e:
        print(f"Simulation failed: {e}")

    
    print("Simulation completed.")

















run_simulation()