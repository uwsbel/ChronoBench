import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random


chrono.SetChronoDataPath("./")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


class ParticleEmitter(chrono.ChBehavior):
    def __init__(self, body, rate, min_radius, max_radius):
        chrono.ChBehavior.__init__(self, body)
        self.rate = rate
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.time_since_last_emission = 0

    def DoStep(self, delta_t):
        self.time_since_last_emission += delta_t
        if self.time_since_last_emission >= 1.0 / self.rate:
            self.time_since_last_emission = 0
            self.emit_particle()

    def emit_particle(self):
        
        sphere = chrono.ChBodyEasySphere(0.1, 1000, True, True)
        radius = random.uniform(self.min_radius, self.max_radius)
        sphere.SetMass(random.uniform(0.1, 1.0))
        sphere.SetPos(chrono.ChVectorD(random.uniform(-2, 2), random.uniform(0, 2), random.uniform(-2, 2)))
        sphere.SetWvel(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
        sphere.SetCollide(True)
        system.Add(sphere)



container = chrono.ChBodyEasyBox(4, 1, 4, 1000, True, True)
container.SetPos(chrono.ChVectorD(0, -1, 0))
container.SetCollide(True)
system.Add(container)


emitter = ParticleEmitter(container, 10, 0.05, 0.15)
emitter.Enable()


class AttractionForce(chrono.ChForce):
    def __init__(self, body1, body2, strength):
        chrono.ChForce.__init__(self, "Attraction")
        self.body1 = body1
        self.body2 = body2
        self.strength = strength

    def Apply(self, delta_t):
        pos1 = self.body1.GetPos()
        pos2 = self.body2.GetPos()
        distance = chrono.ChVectorD(pos2 - pos1).Norm()

        if distance > 0.1:  
            force_magnitude = self.strength / (distance * distance)
            force_direction = chrono.ChVectorD(pos2 - pos1).Normalize()
            force = force_magnitude * force_direction

            self.body1.ApplyForce(force)
            self.body2.ApplyForce(-force)  



def create_attraction_forces():
    bodies = system.Get_BodyList()
    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            force = AttractionForce(bodies[i], bodies[j], 10)  
            system.AddForce(force)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Attraction Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 1, -3))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(0.005)
    emitter.DoStep(0.005)

    
    system.RemoveAllForces()
    create_attraction_forces()