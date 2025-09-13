import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


np.random.seed(42)


sys = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(sys, 'PyChrono Gravitational Attraction', chronoirr.dimension2du(800, 600))


application.SetTimestep(0.01)
application.SetTryRealtime(True)


def create_particle():
    
    shape = np.random.choice(['sphere', 'box', 'cylinder'])
    
    
    if shape == 'sphere':
        particle = chrono.ChBodyEasySphere(0.1,  
                                           1,    
                                           True,  
                                           True)  
    elif shape == 'box':
        particle = chrono.ChBodyEasyBox(0.2,  
                                        1,    
                                        True,  
                                        True)  
    else:
        particle = chrono.ChBodyEasyCylinder(0.1,  
                                             0.2,  
                                             1,    
                                             True,  
                                             True)  
    
    
    particle.SetPos(chrono.ChVectorD(np.random.uniform(-1, 1),
                                     np.random.uniform(-1, 1),
                                     np.random.uniform(-1, 1)))
    
    
    particle.SetVel(chrono.ChVectorD(np.random.uniform(-0.5, 0.5),
                                     np.random.uniform(-0.5, 0.5),
                                     np.random.uniform(-0.5, 0.5)))
    
    
    particle.SetRot(chrono.Q_from_AngZ(np.random.uniform(0, 2 * np.pi)))
    
    return particle


for _ in range(10):
    particle = create_particle()
    sys.Add(particle)


class GravitationalForce(chrono.ChForce):
    def __init__(self):
        super().__init__()
        self.SetF_x(chrono.ChFunction_Const(0))
        self.SetF_y(chrono.ChFunction_Const(0))
        self.SetF_z(chrono.ChFunction_Const(0))
    
    def Update(self, time):
        
        for i in range(sys.GetNbodies()):
            for j in range(i + 1, sys.GetNbodies()):
                body_i = sys.GetBody(i)
                body_j = sys.GetBody(j)
                distance = (body_i.GetPos() - body_j.GetPos()).Length()
                force = 0.1 / (distance ** 2)  
                direction = (body_j.GetPos() - body_i.GetPos()).GetNormalized()
                self.SetF_x(chrono.ChFunction_Const(force * direction.x()))
                self.SetF_y(chrono.ChFunction_Const(force * direction.y()))
                self.SetF_z(chrono.ChFunction_Const(force * direction.z()))
                body_i.AddForce(self)
                body_j.AddForce(-self)


force = GravitationalForce()
sys.Add(force)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()