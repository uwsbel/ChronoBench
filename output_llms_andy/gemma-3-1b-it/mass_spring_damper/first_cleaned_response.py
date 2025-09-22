import pychrono
import pychrono.core as chrono
import pychrono.visual as visual
import numpy as np
import matplotlib.pyplot as plt
from pychrono.visual.visual_utils import visualize_scene

def simulate_mass_spring_damper(initial_mass=1.0, spring_constant=0.5, damping=0.01):
    

    
    env = chrono.Environment()

    
    system = chrono.System(
        name="MassSpringDamper",
        initial_mass=initial_mass,
        spring_constant=spring_constant,
        damping=damping
    )

    
    scene = visual.Scene()
    scene.add_camera(visual.Camera(width=800, height=600, focal_length=600))  
    scene.add_object(visual.Object(name="Mass", position=(0, 0, 0), size=1, color="white"))
    scene.add_object(visual.Object(name="Spring", position=(0, 0, 0), size=1, color="blue"))
    scene.add_object(visual.Object(name="Ground", position=(0, 0, 1), size=1, color="black"))

    
    env.add_object(system, scene)

    
    env.run()

    
    visual.render(scene, env)

    print("Simulation complete.")


if __name__ == "__main__":
    simulate_mass_spring_damper()