import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.particlefactory as chrono_particlefactory
import random


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemSMC()


factory = chrono_particlefactory.ChParticleFactory()


emitter = chrono_particlefactory.ChParticleEmitter()


factory.SetParticleCreator(chrono_particlefactory.ChRandomParticleCreator(
    chrono_particlefactory.ChRandomShapeCreator(
        chrono_particlefactory.ChRandomParticleCreator.ParticleType(chrono_particlefactory.ChRandomParticleCreator.ParticleType_SPHERE, 
                                                                    chrono_particlefactory.ChRandomParticleCreator.ParticleType_BOX, 
                                                                    chrono_particlefactory.ChRandomParticleCreator.ParticleType_ELLIPSOID),
        [0.4, 0.2, 0.1],  
        chrono.ChVector3(0.2, 0.2, 0.2),  
        chrono.ChVector3(0.5, 0.5, 0.5)   
    ),
    chrono_particlefactory.ChRandomPositionCreator(
        chrono.ChCoordsys(chrono.ChVector3(0, 0, 0), chrono.QUNIT),
        chrono.ChCoordsys(chrono.ChVector3(1, 1, 1), chrono.QUNIT)
    ),
    chrono_particlefactory.ChRandomVelocityCreator(
        chrono.ChVector3(-0.5, -0.5, -0.5),
        chrono.ChVector3(0.5, 0.5, 0.5)
    )
))


emitter.SetParticleFactory(factory)


sys.Add(emitter)


class CustomGravity(chrono.ChForce):
    def __init__(self):
        super().__init__()

    def Update(self, time):
        for body in sys.Get_bodylist():
            if isinstance(body, chrono.ChBody):
                
                force = -body.GetPos() * 0.1
                body.AddForce(force)


custom_gravity = CustomGravity()


sys.AddForce(custom_gravity)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Emitter Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3(0, 2, -4))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    emitter.EmitParticles(0.01)