import PyChrono as pc
import numpy as np
from pychrono.objects import *
from pychrono.systems import *
from pychrono.emitters import *
from pychrono.render import *
from pychrono.util import *


pc.init()


sim = pc.Simulation(timestep=1.0)


particles = ParticleSystem()
system = particles.system
system.position = np.array([0, 0, 0])
system.velocity = np.zeros(3)
system.force = np.zeros(3)


class GravitationalForce(Component):
    def __init__(self):
        super().__init__()
        self.distance = 5.0  
        
    def compute(self, particle):
        pos = particle.position
        other_pos = system.position  
        distance = np.linalg.norm(pos - other_pos)
        if distance < self.distance:
            force = -10.0 * (pos - other_pos) / distance  
        else:
            force = np.zeros(3)
        return force

system.force.add(GravitationalForce())


emitter = ParticleEmitter(particles, shape_func=lambda: pc.Sphere(r=0.1, offset=particle.position), 
                        properties=ParticleProperties(
                            color=(0, 0, 1),
                            size=0.1,
                            mass=0.1,
                            angular_momentum=np.random.rand(3) * 0.1
                        ))


renderer = Renderer()
renderer.setSize(800, 600)
renderer.setAntialias(16)
cam = camera.Perspective(45, 800/600, 0.1, 1000)
renderer.camera = cam


system.num_particles = 1000


while True:
    
    t = sim.time
    pos = system.position + (np.random.rand(3) - 0.5) * 5.0
    vel = np.random.rand(3) * 0.1 - 0.05
    force = system.force.compute(system.particles[0])
    system.particle.emit(
        position=pos,
        velocity=vel,
        force=force,
        time=t
    )
    
    
    sim.update()
    
    
    renderer.begin()
    cam.position = np.array([5, 5, 5])
    renderer.draw()
    renderer.end()
    
    
    if renderer.window.is_closed():
        break

pc.quit()