import pychrono as chrono
import pychrono.irrlicht as irr
import random


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()


class ParticleEmitter:
    def __init__(self, system):
        self.system = system

    def emit_particle(self):
        
        shape = random.choice([chrono.ChBodyEasySphere, chrono.ChBodyEasyBox, chrono.ChBodyEasyCylinder])

        
        body = shape(1, 1000, True, True)

        
        body.SetPos(chrono.ChVectorD(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(10, 20)))
        body.SetPos_dt(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
        body.SetRot(chrono.ChQuaternionD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))

        
        self.system.Add(body)


emitter = ParticleEmitter(my_system)


for _ in range(100):
    emitter.emit_particle()


class GravitationalForce(chrono.ChForce):
    def __init__(self, body, strength):
        super(GravitationalForce, self).__init__()
        self.body = body
        self.strength = strength

    def Compute(self, time, A, B, state_x, state_w):
        
        force = -self.strength / self.body.GetPos().Length2() * self.body.GetPos()
        return force


for body in my_system.Get_bodylist():
    force = GravitationalForce(body, 1000)
    body.AddForce(force)


app = irr.ChIrrApp(my_system, 'Particle Gravitation', irr.dimension2du(1024, 768))
app.AddTypicalLights()
app.AddCamera(irr.vector3df(0, 0, -20))
app.AddSkyBox()
app.AddLogo()
app.AddLightWithShadow(irr.vector3df(20, 20, 20), irr.vector3df(0, 0, 0), 20, 50, 1024, 2)
app.SetTimestep(0.01)


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()