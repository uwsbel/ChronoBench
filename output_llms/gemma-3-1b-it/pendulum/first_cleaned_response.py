import pychrono
import pychrono.core as pc
import pychrono.events as ce
import pychrono.visuals as v


g = 9.81  
mass = 1.0  
length = 1.0  
damping = 0.1  
initial_angle = 0.2  
initial_angular_velocity = 0.0  
time_step = 0.01  
simulation_duration = 10.0  


try:
    chrono.init()
    print("PyChrono environment initialized.")
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()




ground_body = pc.GroundBody()
ground_body.mass = 1.0
ground_body.position = (0, 0)  
ground_body.rotation = pc.Rotation(0, 0, 0) 


pendulum = pc.Pendulum(mass, length, damping=damping)
pendulum.position = (0, initial_angle)
pendulum.velocity = (initial_angular_velocity, 0)  



visualizer = v.Visualizer()


canvas = visualizer.create_canvas(width=800, height=600)


try:
    for t in range(simulation_duration):
        
        angle_change = pendulum.velocity[0] * t
        pendulum.position = pendulum.position + angle_change

        
        pendulum.velocity = pendulum.velocity + pendulum.velocity * (g / length) * t

        
        visualizer.render(pendulum, canvas)

        
        print(f"Time: {t:.2f}, Angle: {pendulum.angle:.2f}, Velocity: {pendulum.velocity}")

except Exception as e:
    print(f"Error during simulation: {e}")

finally:
    
    visualizer.destroy()
    visualizer.close()
    chrono.shutdown()