import pychrono
import pychrono.core as chrono
import pychrono.visual as visual
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Simulation Parameters ---
grid_size = 100  # Adjust for simulation detail
time_step = 0.01  # Smaller for higher accuracy
simulation_duration = 10  # Seconds
vehicle_mass = 1000  # kg (Adjust for vehicle weight)
vehicle_acceleration = 2  # m/s^2 (Adjust for vehicle speed)
friction_coefficient = 0.7  # Adjust for friction
restitution_coefficient = 0.7  # Adjust for restitution
terrain_friction = 0.8  # Adjust for terrain friction
terrain_restitution = 0.6  # Adjust for terrain restitution

# --- Terrain Setup ---
terrain_heightmap = np.zeros((grid_size, grid_size))
terrain_heightmap[10:20, 10:20] = 1.0  # Create a simple hill for demonstration
terrain_heightmap[10:20, 10:20] = 0.5  # Create a valley

# --- Vehicle Setup ---
vehicle_x = 0
vehicle_y = 0
vehicle_vx = 0
vehicle_vy = 0

# --- Driver System ---
driver_x = 0
driver_y = 0
driver_ax = 0
driver_ay = 0

# --- Initial Conditions ---
initial_position = (vehicle_x, vehicle_y)
initial_velocity = (vehicle_vx, vehicle_vy)

# --- Visualization Setup ---
fig = visual.Figure(width=800, height=600)
ax = fig.add_subplot(111, projection='3d')
camera_x = 0
camera_y = 0
camera_z = 0
camera_fov = 60
camera_aspect = 1.0
camera_tilt = 0.0
camera_zoom = 1.0

# --- Initialize Camera ---
camera_frame = chrono.Frame(
    x=camera_x,
    y=camera_y,
    z=camera_z,
    fov=camera_fov,
    aspect=camera_aspect,
    tilt=camera_tilt,
    zoom=camera_zoom
)
ax.add_collection(camera_frame)

# --- Simulation Loop ---
for t in range(simulation_duration):
    # --- Update Vehicle ---
    vehicle_vx += vehicle_acceleration * time_step
    vehicle_vy += vehicle_acceleration * time_step

    # --- Update Terrain ---
    terrain_heightmap[vehicle_x, vehicle_y] += terrain_vx * time_step
    terrain_heightmap[vehicle_x, vehicle_y] += terrain_vy * time_step

    # --- Update Driver ---
    driver_x += driver_ax * time_step
    driver_y += driver_ay * time_step

    # --- Update Visualization ---
    # Update the scene with the current vehicle position and velocity
    ax.set_xlim(vehicle_x - 1, vehicle_x + 1)
    ax.set_ylim(vehicle_y - 1, vehicle_y + 1)
    ax.set_zlim(0, 1)
    ax.set_title(f"Time: {t}")

    # --- Update Animation ---
    ani = animation.FuncAnimation(fig, lambda t: ax.set_data(t, ax.set_xlim()),
                                 interval=10,
                                 frames=100)  # Adjust frames per second

    # --- Display the Animation ---
    plt.show()

    # --- Save the Visualization (optional) ---
    # np.save("simulation_visualization.png",ani.as_array())
    # print("Visualization saved to simulation_visualization.png")


print("Simulation complete.")