import pychrono
import pychrono.core as pc
import pychrono.visual as cv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def simulate_beam(num_elements=100, gravity=9.81, initial_position=(0, 0, 0)):
    

    
    env = pc.Environment()

    
    
    elements = []
    for i in range(num_elements):
        
        x, y, z = initial_position
        elements.append(pc.Point(x, y, z))

    
    env.set_physics_system(pc.System(elements))
    env.set_gravity(gravity)
    env.set_time(pc.Time(0)) 

    
    for t in range(pc.Time(0), pc.Time(pc.Time(0) + 10)):  
        
        env.update_physics()

        
        
        fig = cv.figure()
        ax = fig.add_subplot(111, projection='3d')

        
        for element in elements:
            
            deformation = np.array([element.x, element.y, element.z])
            ax.plot(deformation, color='blue')  
        
        
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([-1.5, 1.5])
        
        
        cv.show()

    env.close()

if __name__ == '__main__':
    simulate_beam(num_elements=100, gravity=9.81, initial_position=(0, 0, 0))