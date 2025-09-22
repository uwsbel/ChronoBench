import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.particlefactory as chrono_particle
import random


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemSMC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
sys.SetStep(1e-4)


particle_factory = chrono_particle.ChParticleEmitterFactory()


emitter = chrono_particle.ChParticleEmitter(
    particle_factory,
    sys,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),
    100,  
    10,   
    0.01, 
    chrono.ChVectorD(0, 0.1, 0),  
    chrono.ChVectorD(0.1, 0.1, 0.1),  
)


def custom_gravity(body1, body2):
    G = 6.67430e-11  
    pos1 = body1.GetPos()
    pos2 = body2.GetPos()
    distance = (pos2 - pos1).Length()
    if distance > 1e-6:
        direction = (pos2 - pos1) / distance
        force = G * body1.GetMass() * body2.GetMass() / (distance ** 2) * direction
        body1.AddForce(force)
        body2.AddForce(-force)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Particle Emitter Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, -4))
vis.AttachSystem(sys)


while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    emitter.Update(chrono.ChTime().GetStep())

    
    for body1 in sys.Get_bodylist():
        for body2 in sys.Get_bodylist():
            if body1 != body2:
                custom_gravity(body1, body2)

    
    sys.DoStepDynamics(chrono.ChTime().GetStep())

    vis.EndScene()


chrono.SetChronoDataPath("/path/to/chrono/data/")

chrono.SetChronoDataPath("/usr/local/share/chrono/data/")

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.particlefactory as chrono_particle
import random




while vis.Run():
    

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.particlefactory as chrono_particle
import random


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemSMC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
sys.SetStep(1e-4)


particle_factory = chrono_particle.ChParticleEmitterFactory()


emitter = chrono_particle.ChParticleEmitter(
    particle_factory,
    sys,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),
    100,  
    10,   
    0.01, 
    chrono.ChVectorD(0, 0.1, 0),  
    chrono.ChVectorD(0.1, 0.1, 0.1),  
)


def custom_gravity(body1, body2):
    G = 6.67430e-11  
    pos1 = body1.GetPos()
    pos2 = body2.GetPos()
    distance = (pos2 - pos1).Length()
    if distance > 1e-6:
        direction = (pos2 - pos1) / distance
        force = G * body1.GetMass() * body2.GetMass() / (distance ** 2) * direction
        body1.AddForce(force)
        body2.AddForce(-force)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Particle Emitter Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, -4))
vis.AttachSystem(sys)


while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    emitter.Update(chrono.ChTime().GetStep())

    
    for body1 in sys.Get_bodylist():
        for body2 in sys.Get_bodylist():
            if body1 != body2:
                custom_gravity(body1, body2)

    
    sys.DoStepDynamics(chrono.ChTime().GetStep())

    vis.EndScene()