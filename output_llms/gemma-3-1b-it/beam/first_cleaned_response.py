import pychrono
import pychrono.physics as ph
import pychrono.visuals as v
import pychrono.widgets as wg
import matplotlib.pyplot as plt
import numpy as np


num_nodes = 100  
beam_length = 1.0  
beam_thickness = 0.01  
num_beams = 5  
material_density = 1.0  
material_viscosity = 0.01 

initial_position = [0.0, 0.0, 0.0]  
initial_velocity = [1.0, 0.0, 0.0]  
initial_force = [0.0, 0.0, 0.0]  
initial_time = 0.0 


chrono = pychrono.chrono.Chrono()
chrono.set_physics_system(ph.BeamSystem(num_nodes, beam_length, beam_thickness))
chrono.set_physics_mode(ph.StaticMode()) 


v_init = v.Plotter()
v_init.set_title("Beam Finite Element Simulation")
v_init.set_zoom(1.0)
v_init.set_view_mode("3D")  
v_init.set_background_color("white")


try:
    while True:
        
        chrono.update_physics(chrono.physics_mode(), chrono.physics_system(), chrono.physics_mode())

        
        v_init.draw_mesh(chrono.physics_system(), chrono.physics_mode())
        v_init.draw_lines(chrono.physics_system(), chrono.physics_mode()) 
        v_init.draw_text(f"Time: {chrono.time_since_epoch()}", 0.05, v.Text.AlignHCenter)
        v_init.draw_text(f"Position: {chrono.physics_system()[0]}", 0.05, v.Text.AlignLCenter)
        v_init.draw_text(f"Velocity: {chrono.physics_system()[1]}", 0.05, v.Text.AlignLCenter)
        v_init.draw_text(f"Force: {chrono.physics_system()[2]}", 0.05, v.Text.AlignRCenter)

        
        v_init.update()

        
        wg.show_simulation(v_init)

        
        plt.pause(0.01)  

except KeyboardInterrupt:
    print("Simulation interrupted.")
    plt.close(v_init)
    chrono.stop()