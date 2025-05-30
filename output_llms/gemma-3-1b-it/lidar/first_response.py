import pychrono
import pychrono.objects
import pychrono.physics
import pychrono.sensors
import pychrono.visuals
import pychrono.sensors.lidar
import pychrono.visuals.lidar_visuals
import pychrono.sensors.lidar.noise_filters
import pychrono.visuals.lidar_data
import numpy as np
import matplotlib.pyplot as plt

# --- 1. Initialization & Core Components ---

# Define the mesh (Wavefront .obj file)
mesh_file = "path/to/your/mesh.obj"  # Replace with your file path
try:
    mesh = pychrono.objects.Mesh.from_obj(mesh_file)
except Exception as e:
    print(f"Error loading mesh: {e}")
    return  # Exit if the mesh can't be loaded

# Define the body (Triangular Mesh)
body_type = "triangular"
body_position = [0, 0, 1]  # Initial position
body_velocity = [0, 0, 1] # Initial velocity
body_force = [0, 0, 1] # Initial force (gravity)

# --- 2. Sensor Manager & Lidar ---

# Sensor Manager Setup
sensor_manager = pychrono.sensors.sensor_manager.SensorManager()
sensor_manager.add_sensor(pychrono.sensors.Lidar(mesh=mesh, position=body_position, velocity=body_velocity, force=body_force))

# Noise Filters (Basic - adjust as needed)
noise_filter = pychrono.visuals.lidar_visuals.NoiseFilter(
    noise_type="gaussian",
    sigma=1.0
)
lidar_data = sensor_manager.lidar_data

# --- 3. Simulation Parameters ---

simulation_time = 10  # Simulation duration in seconds
simulation_steps = 100
simulation_step_interval = 1.0 / simulation_time  # Time between steps

# --- 4. Simulation Loop ---

def run_simulation():
    print("Starting simulation...")
    try:
        # Initialize PyChrono
        pychrono.init()

        # Simulation Loop
        for i in range(simulation_steps):
            # 1. Update Lidar Data
            lidar_data = sensor_manager.lidar_data
            lidar_buffer = lidar_data.get_buffer()

            # 2. Visualization
            visual = pychrono.visuals.lidar_visuals.LidarVisual(lidar_buffer)
            visual.set_position(lidar_buffer.position)
            visual.set_scale(1.0)  # Adjust scale as needed

            # 3.  Print Buffer Data (for debugging)
            print(f"Step {i+1}: Lidar Buffer Data")
            print(lidar_buffer.buffer)

            # 4.  Update the simulation
            pychrono.update_step()

    except Exception as e:
        print(f"Error during simulation: {e}")

    finally:
        pychrono.cleanup()  # Clean up resources

if __name__ == "__main__":
    run_simulation()