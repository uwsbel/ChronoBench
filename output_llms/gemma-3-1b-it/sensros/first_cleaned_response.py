import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import Chrono
from pychrono.core.sensor import Sensor
from pychrono.core.simulation import Simulation
from pychrono.core.physics import Force
from pychrono.core.utils import get_time_point


simulation_name = "GroundBodySimulation"
sensor_manager_name = "GroundSensorManager"
simulation_duration = 100  
time_step = 0.1  
num_sensors = 3  
sensor_id_map = {} 

try:
    chrono = Chrono(simulation_name, sensor_manager_name)
    print(f"PyChrono simulation '{simulation_name}' initialized.")
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()




ground_body = {
    'id': 'ground_body',
    'position': np.array([0, 0, 0]),
    'velocity': np.array([0, 0, 0]),
    'mass': 10.0,
    'friction': 0.1,
    'sensor_id': 'ground_body_sensor' 
}


camera = {
    'id': 'camera_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'camera_1_sensor'
}


lidar = {
    'id': 'lidar_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'lidar_1_sensor'
}


gps = {
    'id': 'gps_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'gps_1_sensor'
}


accelerometer = {
    'id': 'accelerometer_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'accelerometer_1_sensor'
}


gyro = {
    'id': 'gyro_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'gyro_1_sensor'
}


magnetometer = {
    'id': 'magnetometer_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'magnetometer_1_sensor'
}


sensor_manager = SensorManager(sensor_id_map)


def update_simulation():
    global ground_body
    global camera
    global lidar
    global gps
    global accelerometer
    global gyroscope
    global magnetometer

    
    ground_body['velocity'] = np.array([0, 0, 0])
    ground_body['position'] = ground_body['position'] + np.array(camera['position'])
    ground_body['position'] = ground_body['position'] + np.array(lidar['position'])
    ground_body['position'] = ground_body['position'] + np.array(gps['position'])
    ground_body['position'] = ground_body['position'] + np.array(accelerometer['position'])
    ground_body['position'] = ground_body['position'] + np.array(gyro['position'])
    ground_body['position'] = ground_body['position'] + np.array(magnetometer['position'])

    
    ground_body['velocity'] = ground_body['velocity'] + 0.01 * np.array([0, 0, -9.81])  

    
    time_point = Chrono.TimePoint(time_step)
    chrono.advance(time_point)

    
    camera['position'] = ground_body['position'] + np.array(camera['position'])
    camera['orientation'] = ground_body['position'] + np.array(camera['orientation'])
    camera['orientation'] = ground_body['position'] + np.array(lidar['orientation'])
    camera['orientation'] = ground_body['position'] + np.array(gps['orientation'])
    camera['orientation'] = ground_body['position'] + np.array(accelerometer['orientation'])
    camera['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    camera['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    
    lidar['position'] = ground_body['position'] + np.array(lidar['position'])
    lidar['orientation'] = ground_body['position'] + np.array(lidar['orientation'])
    lidar['orientation'] = ground_body['position'] + np.array(gps['orientation'])
    lidar['orientation'] = ground_body['position'] + np.array(accelerometer['orientation'])
    lidar['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    lidar['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    
    gps['position'] = ground_body['position'] + np.array(gps['position'])
    gps['orientation'] = ground_body['position'] + np.array(gps['orientation'])
    gps['orientation'] = ground_body['position'] + np.array(gps['orientation'])
    gps['orientation'] = ground_body['position'] + np.array(accelerometer['orientation'])
    gps['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    gps['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    
    accelerometer['position'] = ground_body['position'] + np.array(accelerometer['position'])
    accelerometer['orientation'] = ground_body['position'] + np.array(accelerometer['orientation'])
    accelerometer['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    accelerometer['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    
    gyro['position'] = ground_body['position'] + np.array(gyro['position'])
    gyro['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    gyro['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    
    magnetometer['position'] = ground_body['position'] + np.array(magnetometer['position'])
    magnetometer['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])
    magnetometer['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    magnetometer['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    
    plt.pause(0.01)  
    print(f"Simulation time: {time_step:.2f} seconds")



update_simulation()
print("Simulation complete.")