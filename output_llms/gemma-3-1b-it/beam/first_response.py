import pychrono
import pychrono.physics as ph
import pychrono.visuals as v
import pychrono.widgets as wg
import matplotlib.pyplot as plt
import numpy as np

# --- Simulation Parameters ---
num_nodes = 100  # Number of nodes in the mesh
beam_length = 1.0  # Length of the beams
beam_thickness = 0.01  # Thickness of the beams
num_beams = 5  # Number of beams in the mesh
material_density = 1.0  # Density of the material (adjust for different behaviors)
material_viscosity = 0.01 #Viscosity value for fluid simulation
# --- Initial Conditions ---
initial_position = [0.0, 0.0, 0.0]  # Initial position of the beam
initial_velocity = [1.0, 0.0, 0.0]  # Initial velocity of the beam
initial_force = [0.0, 0.0, 0.0]  # Initial force applied to the beam
initial_time = 0.0 # Start time of the simulation

# --- PyChrono Setup ---
chrono = pychrono.chrono.Chrono()
chrono.set_physics_system(ph.BeamSystem(num_nodes, beam_length, beam_thickness))
chrono.set_physics_mode(ph.StaticMode()) # Or 'DynamicMode' if you need dynamic behavior

# --- Visualization ---
v_init = v.Plotter()
v_init.set_title("Beam Finite Element Simulation")
v_init.set_zoom(1.0)
v_init.set_view_mode("3D")  # Set to 3D for better visualization
v_init.set_background_color("white")

# --- Simulation Loop ---
try:
    while True:
        # --- Update Physics ---
        chrono.update_physics(chrono.physics_mode(), chrono.physics_system(), chrono.physics_mode())

        # --- Rendering ---
        v_init.draw_mesh(chrono.physics_system(), chrono.physics_mode())
        v_init.draw_lines(chrono.physics_system(), chrono.physics_mode()) # Add lines for clarity
        v_init.draw_text(f"Time: {chrono.time_since_epoch()}", 0.05, v.Text.AlignHCenter)
        v_init.draw_text(f"Position: {chrono.physics_system()[0]}", 0.05, v.Text.AlignLCenter)
        v_init.draw_text(f"Velocity: {chrono.physics_system()[1]}", 0.05, v.Text.AlignLCenter)
        v_init.draw_text(f"Force: {chrono.physics_system()[2]}", 0.05, v.Text.AlignRCenter)

        # --- Visualization Update ---
        v_init.update()

        # ---  Display the simulation in a window ---
        wg.show_simulation(v_init)

        # --- Wait for a short time ---
        plt.pause(0.01)  # Adjust for desired simulation speed

except KeyboardInterrupt:
    print("Simulation interrupted.")
    plt.close(v_init)
    chrono.stop()