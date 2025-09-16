import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a material for the particles
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.2)
material.SetRestitution(0.5)

# Create an Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(mysystem)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Particle Gravity Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddLight(chrono.ChVectorD(5, 5, 5), chrono.ChColor(1, 1, 1))

# ---------------------------------------------------------------------
#
#  Particle emitter settings
#

class ParticleEmitter:
    def __init__(self, system, pos, rate):
        self.system = system
        self.pos = pos
        self.rate = rate
        self.timer = 0

    def Update(self, dt):
        self.timer += dt
        if self.timer > 1/self.rate:
            self.timer = 0
            self.EmitParticle()

    def EmitParticle(self):
        # Create a particle body
        body = chrono.ChBody()
        body.SetMass(1)
        body.SetPos(self.pos)

        # Randomize shape, velocity, and orientation
        shape = random.choice([chrono.ChSphereShape(), chrono.ChBoxShape(chrono.ChVectorD(0.1, 0.1, 0.1))])
        body.AddAsset(shape)
        body.SetBodyFixed(False)
        body.SetVel(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(0, 2), random.uniform(-1, 1)))
        body.SetRot(chrono.Q_from_AngAxis(random.uniform(0, 2*3.14159), chrono.ChVectorD(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))))

        # Add the particle to the system
        self.system.AddBody(body)

# Create a particle emitter
emitter = ParticleEmitter(mysystem, chrono.ChVectorD(0, 5, 0), 10)

# ---------------------------------------------------------------------
#
#  Custom gravitational force
#

class GravityForce(chrono.ChForce):
    def __init__(self, strength):
        chrono.ChForce.__init__(self)
        self.strength = strength

    def AddForce(self, body, dt):
        for other_body in mysystem.Get_bodylist():
            if body != other_body:
                r = other_body.GetPos() - body.GetPos()
                dist = r.Length()
                force = self.strength * body.GetMass() * other_body.GetMass() / (dist**2)
                body.AddForce(force * r.GetNormalized())

# Create a custom gravity force
gravity_force = GravityForce(10)

# ---------------------------------------------------------------------
#
#  Simulation loop
#

while vis.Run():
    mysystem.DoStepDynamics(0.01)
    emitter.Update(0.01)
    gravity_force.AddForce(mysystem.Get_bodylist()[0], 0.01)
    vis.Render()