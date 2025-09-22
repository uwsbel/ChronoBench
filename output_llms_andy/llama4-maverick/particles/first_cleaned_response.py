import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.particlefactory as chrono_particlefactory
import random


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemSMC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


factory = chrono_particlefactory.ChParticleEmitterFactory()


emitter = factory.CreateParticleEmitter(
    chrono_particlefactory.ChParticleEmitterType_BOX,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_Euler123(chrono.ChVectorD(0, 0, 0))),
    chrono.ChVectorD(0.1, 0.1, 0.1),  
    chrono.ChVectorD(0, 0, 0),  
    0.1,  
    1000,  
    0.01  
)


sys.Add(emitter)


class CustomGravity(chrono.ChCustomForce):
    def __init__(self, body1, body2):
        super().__init__()
        self.body1 = body1
        self.body2 = body2

    def GetF(self, time):
        pos1 = self.body1.GetPos()
        pos2 = self.body2.GetPos()
        dist = (pos2 - pos1).Length()
        if dist > 0:
            force = 0.1 * self.body1.GetMass() * self.body2.GetMass() / (dist * dist)
            direction = (pos2 - pos1).GetNormalized()
            return chrono.ChVectorD(force * direction.x, force * direction.y, force * direction.z)
        else:
            return chrono.ChVectorD(0, 0, 0)


body1 = chrono.ChBodyEasySphere(0.5, 1000, True, True)
body1.SetPos(chrono.ChVectorD(-5, 0, 0))
sys.Add(body1)

body2 = chrono.ChBodyEasySphere(0.5, 1000, True, True)
body2.SetPos(chrono.ChVectorD(5, 0, 0))
sys.Add(body2)


for body in sys.Get_bodylist():
    if body != body1 and body != body2:
        force1 = CustomGravity(body, body1)
        body.AddForce(force1)
        force2 = CustomGravity(body, body2)
        body.AddForce(force2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Particle Attraction Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()