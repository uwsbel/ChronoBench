import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import Chrono
from pychrono.core.sensor import Sensor
from pychrono.core.simulation import Simulation
from pychrono.core.physics import Force
from pychrono.core.utils import get_time_point

# --- Configuration ---
simulation_name = "GroundBodySimulation"
sensor_manager_name = "GroundSensorManager"
simulation_duration = 100  # seconds
time_step = 0.1  # seconds
num_sensors = 3  # Camera, Lidar, GPS, Accelerometer, Gyroscope, Magnetometer
sensor_id_map = {} # Mapping sensor IDs to their respective objects
# --- Initialize PyChrono ---
try:
    chrono = Chrono(simulation_name, sensor_manager_name)
    print(f"PyChrono simulation '{simulation_name}' initialized.")
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()


# --- Define Physical Systems ---
# Ground Body
ground_body = {
    'id': 'ground_body',
    'position': np.array([0, 0, 0]),
    'velocity': np.array([0, 0, 0]),
    'mass': 10.0,
    'friction': 0.1,
    'sensor_id': 'ground_body_sensor' # Assign sensor ID for the ground body
}

# Cameras
camera = {
    'id': 'camera_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'camera_1_sensor'
}

# Lidar
lidar = {
    'id': 'lidar_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'lidar_1_sensor'
}

# GPS
gps = {
    'id': 'gps_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'gps_1_sensor'
}

# Accelerometer
accelerometer = {
    'id': 'accelerometer_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'accelerometer_1_sensor'
}

# Gyroscope
gyro = {
    'id': 'gyro_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'gyro_1_sensor'
}

# Magnetometer
magnetometer = {
    'id': 'magnetometer_1',
    'position': np.array([0, 0, 0]),
    'orientation': np.array([0, 0, 0]),
    'sensor_id': 'magnetometer_1_sensor'
}

# --- Sensor Manager ---
sensor_manager = SensorManager(sensor_id_map)

# --- Simulation Loop ---
def update_simulation():
    global ground_body
    global camera
    global lidar
    global gps
    global accelerometer
    global gyroscope
    global magnetometer

    # Update sensor readings
    ground_body['velocity'] = np.array([0, 0, 0])
    ground_body['position'] = ground_body['position'] + np.array(camera['position'])
    ground_body['position'] = ground_body['position'] + np.array(lidar['position'])
    ground_body['position'] = ground_body['position'] + np.array(gps['position'])
    ground_body['position'] = ground_body['position'] + np.array(accelerometer['position'])
    ground_body['position'] = ground_body['position'] + np.array(gyro['position'])
    ground_body['position'] = ground_body['position'] + np.array(magnetometer['position'])

    # Simulate gravity
    ground_body['velocity'] = ground_body['velocity'] + 0.01 * np.array([0, 0, -9.81])  # Simulate gravity

    # Advance the simulation
    time_point = Chrono.TimePoint(time_step)
    chrono.advance(time_point)

    # Update camera
    camera['position'] = ground_body['position'] + np.array(camera['position'])
    camera['orientation'] = ground_body['position'] + np.array(camera['orientation'])
    camera['orientation'] = ground_body['position'] + np.array(lidar['orientation'])
    camera['orientation'] = ground_body['position'] + np.array(gps['orientation'])
    camera['orientation'] = ground_body['position'] + np.array(accelerometer['orientation'])
    camera['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    camera['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    # Update lidar
    lidar['position'] = ground_body['position'] + np.array(lidar['position'])
    lidar['orientation'] = ground_body['position'] + np.array(lidar['orientation'])
    lidar['orientation'] = ground_body['position'] + np.array(gps['orientation'])
    lidar['orientation'] = ground_body['position'] + np.array(accelerometer['orientation'])
    lidar['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    lidar['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    # Update GPS
    gps['position'] = ground_body['position'] + np.array(gps['position'])
    gps['orientation'] = ground_body['position'] + np.array(gps['orientation'])
    gps['orientation'] = ground_body['position'] + np.array(gps['orientation'])
    gps['orientation'] = ground_body['position'] + np.array(accelerometer['orientation'])
    gps['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    gps['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    # Update accelerometer
    accelerometer['position'] = ground_body['position'] + np.array(accelerometer['position'])
    accelerometer['orientation'] = ground_body['position'] + np.array(accelerometer['orientation'])
    accelerometer['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    accelerometer['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    # Update gyroscope
    gyro['position'] = ground_body['position'] + np.array(gyro['position'])
    gyro['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    gyro['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    # Update magnetometer
    magnetometer['position'] = ground_body['position'] + np.array(magnetometer['position'])
    magnetometer['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])
    magnetometer['orientation'] = ground_body['position'] + np.array(gyro['orientation'])
    magnetometer['orientation'] = ground_body['position'] + np.array(magnetometer['orientation'])

    # --- Visualization ---
    plt.pause(0.01)  # Pause for visualization
    print(f"Simulation time: {time_step:.2f} seconds")

# --- Main Loop ---
# Start the simulation
update_simulation()
print("Simulation complete.")