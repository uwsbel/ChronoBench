import pychrono
import pychrono.core as chrono
import pychrono.visual as visual
import pychrono.utils as utils
import random
import numpy as np

# --- Simulation Parameters ---
terrain_width = 10.0
terrain_height = 5.0
terrain_depth = 2.0
vehicle_speed = 0.5
vehicle_acceleration = 0.3
vehicle_deceleration = 0.2
vehicle_steering_sensitivity = 0.1
vehicle_throttle_sensitivity = 0.05
vehicle_brake_sensitivity = 0.02

# --- Initial Vehicle Setup ---
vehicle_location = (0, 0, 0)
vehicle_orientation = (0, 0, 0)
contact_method = "friction"  # "friction", "elastic", "none"
visualization_type = "raytrace" # "raytrace", "block"

# --- Terrain Setup ---
terrain = visual.RigidTerrain(
    width=terrain_width,
    height=terrain_height,
    depth=terrain_depth,
    texture="terrain.png",  # Replace with your terrain texture
    color=(0.8, 0.8, 0.8) # Light gray terrain
)

# --- Vehicle Properties ---
vehicle_mass = 10.0
vehicle_center_of_mass = (vehicle_location[0], vehicle_location[1], vehicle_location[2])
vehicle_radius = 0.5

# --- Simulation Loop ---
def run_simulation():
    global vehicle_location, vehicle_orientation, contact_method, visualization_type

    try:
        # Initialize PyChrono
        chrono.init()

        # Set simulation parameters
        chrono.set_simulation_parameters(
            time_step=0.01,  # Adjust for desired smoothness
            frame_rate=50,
            gravity=0.0,
            vehicle_mass=vehicle_mass,
            vehicle_center_of_mass=vehicle_center_of_mass,
            vehicle_radius=vehicle_radius,
            vehicle_speed=vehicle_speed,
            vehicle_acceleration=vehicle_acceleration,
            vehicle_deceleration=vehicle_deceleration,
            contact_method=contact_method,
            visualization_type=visualization_type
        )

        # Update the simulation
        while True:
            # Update vehicle dynamics (simplified)
            vehicle_orientation += vehicle_speed * 0.1  # Small angle change
            vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0) # Keep within bounds

            # Apply contact
            if contact_method == "friction":
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)

            elif contact_method == "elastic":
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)

            elif contact_method == "none":
                pass # No contact

            # Update visualization
            visual.update(visualization_type)

            # Display the simulation
            visual.display(vehicle_location, vehicle_orientation, vehicle_radius)

            # Wait for the next frame
            time.sleep(0.01)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Clean up
        chrono.cleanup()

if __name__ == "__main__":
    run_simulation()