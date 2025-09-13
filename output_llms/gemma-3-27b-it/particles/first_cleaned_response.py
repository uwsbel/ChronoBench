import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


chrono.SetChronoDataPath("./")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


class ParticleEmitter(chrono.ChBehavior):
    def __init__(self, body, rate, min_radius, max_radius):
        chrono.ChBehavior.__init__(self)
        self.body = body
        self.rate = rate
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.time_since_last_emission = 0.0

    def DoStep(self, step):
        time = step.GetTime()
        dt = step.GetStepSize()
        self.time_since_last_emission += dt

        while self.time_since_last_emission > (1.0 / self.rate):
            
            particle = chrono.ChBodyEasy()
            particle.SetBodyFixed(False)
            particle.SetCollide(True)
            particle.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())

            
            radius = random.uniform(self.min_radius, self.max_radius)
            particle.AddSphere(radius)
            particle.SetMass(1.0)
            particle.SetInertia(0.5 * particle.GetMass() * radius * radius)

            
            x = random.uniform(-1.0, 1.0)
            y = random.uniform(0.0, 1.0)
            z = random.uniform(-1.0, 1.0)
            particle.SetPos(chrono.ChVectorD(x, y, z))

            
            vx = random.uniform(-1.0, 1.0)
            vy = random.uniform(0.0, 1.0)
            vz = random.uniform(-1.0, 1.0)
            particle.SetWVel(chrono.ChVectorD(vx, vy, vz))

            
            particle.SetRot(chrono.ChQuaternionD(random.uniform(0, 2*math.pi), random.uniform(0, 2*math.pi), random.uniform(0, 2*math.pi)))

            
            system.Add(particle)

            self.time_since_last_emission -= (1.0 / self.rate)


emitter_body = chrono.ChBodyEasy()
emitter_body.SetBodyFixed(True)
emitter_body.SetPos(chrono.ChVectorD(0, 0, 0))
emitter_body.AddBox(0.2, 0.2, 0.2)
system.Add(emitter_body)


emitter = ParticleEmitter(emitter_body, 10, 0.05, 0.1)
emitter_body.AddBehavior(emitter)



class AttractiveForce(chrono.ChForce):
    def __init__(self, strength):
        chrono.ChForce.__init__(self)
        self.strength = strength

    def Apply(self, step):
        for body in system.Get_BodyList():
            for other_body in system.Get_BodyList():
                if body != other_body:
                    distance = chrono.ChVectorD(body.GetPos() - other_body.GetPos()).Length()
                    if distance > 0.01: 
                        force_magnitude = self.strength / (distance * distance)
                        force_direction = chrono.ChVectorD(other_body.GetPos() - body.GetPos()).Normalize()
                        force = force_magnitude * force_direction
                        body.ApplyForce(force)



attractive_force = AttractiveForce(100.0)
system.AddForce(attractive_force)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Attraction')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 1.5, -3))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)