import pychrono
import numpy as np
from pychrono.core import System, Event
from pychrono.core import Observer
from pychrono.core import Station
from pychrono.core import TimeStep
from pychrono.core import Event

# --- Configuration ---
# Vehicle Parameters
vehicle_id = 1
vehicle_type = "HMMWV"
contact_method = "Contact"  # "Contact", "Push"
engine_type = "Diesel"
tire_model = "All-Terrain"
vehicle_mass = 15000  # kg
vehicle_length = 8.0  # meters
vehicle_width = 3.0  # meters
vehicle_height = 2.5  # meters
vehicle_radius = 0.3 # meters
terrain_friction = 0.9  # m/s^2
terrain_restitution = 0.7 # m/s^2
vehicle_speed = 10.0  # m/s

# Terrain Parameters
terrain_height = 1.0 # meters
terrain_friction_factor = 0.8
terrain_restitution_factor = 0.6

# Driver Parameters
driver_id = 2
driver_control_method = "PID"
driver_acceleration = 2.0
driver_steering = 1.0
driver_throttle = 0.5

# --- Simulation Setup ---
# Initialize PyChrono System
system = System(
    station=Station(
        time_step=1.0,
        time_step_unit="s"
    ),
    vehicle_id=vehicle_id,
    vehicle_type=vehicle_type,
    contact_method=contact_method,
    engine_type=engine_type,
    tire_model=tire_model,
    vehicle_mass=vehicle_mass,
    vehicle_length=vehicle_length,
    vehicle_width=vehicle_width,
    vehicle_height=vehicle_height,
    vehicle_radius=vehicle_radius,
    terrain_friction=terrain_friction,
    terrain_restitution=terrain_restitution,
    vehicle_speed=vehicle_speed
)

# --- Time Step Initialization ---
time_step = 1.0  # seconds
time_step_count = 0
time_step_history = []

# --- Simulation Loop ---
while time_step_count < 100:  # Simulate for 100 time steps
    # --- Update Vehicle State ---
    # 1.  Update Vehicle Speed
    vehicle_speed = max(0, min(vehicle_speed, 30.0))  # Limit speed

    # 2.  Update Vehicle Position
    vehicle_position = system.get_position()
    vehicle_position = np.array(vehicle_position)

    # 3.  Update Terrain Position
    terrain_position = system.get_position()
    terrain_position = np.array(terrain_position)

    # 4.  Update Driver Input
    driver_input = system.get_input()
    driver_input = np.array(driver_input)

    # 5.  Update Vehicle State (Based on Driver Input)
    vehicle_state = system.get_state()
    vehicle_state = np.array(vehicle_state)

    # 6.  Update Terrain Position (Based on Driver Input)
    terrain_state = system.get_state()
    terrain_state = np.array(terrain_state)

    # 7.  Update Vehicle Position (Based on Terrain State)
    vehicle_position = np.clip(vehicle_position, -10.0, 10.0)  # Clip to prevent out-of-bounds

    # 8.  Update Terrain Position (Based on Vehicle Position)
    terrain_position = np.clip(terrain_position, -10.0, 10.0)

    # --- ROS Integration ---
    # 1.  Register Handler
    handler = system.register_handler(
        driver_id,
        driver_control_method,
        driver_acceleration,
        driver_steering,
        driver_throttle,
        driver_input,
        vehicle_state,
        terrain_state,
        time_step_count
    )

    # 2.  Publish Data
    system.publish_data(handler)

    # --- Time Step Increment ---
    time_step_count += 1.0
    time_step_history.append(time_step)

    # --- Logging (Optional) ---
    print(f"Time Step: {time_step}, Time Step Count: {time_step_count}")

# --- End of Simulation ---
# ---  Cleanup ---
# System.stop()
# system.close()