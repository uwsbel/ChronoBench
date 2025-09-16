import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random


chrono.SetChronoDataPath("../chrono_data/")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


class ParticleEmitter(chrono.ChBehavior):
    def __init__(self, body, rate, min_radius, max_radius):
        chrono.ChBehavior.__init__(self, body)
        self.rate = rate
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.time_since_last_emission = 0

    def DoStep(self, step):
        time = step.GetTime()
        dt = step.GetStepSize()
        self.time_since_last_emission += dt

        if self.time_since_last_emission >= 1.0 / self.rate:
            self.time_since_last_emission = 0

            
            particle = chrono.ChBodyEasy()
            particle.SetBodyFixed(False)
            particle.SetMass(0.1)  

            
            radius = random.uniform(self.min_radius, self.max_radius)
            sphere_asset = chrono.ChSphereShape()
            sphere_asset.GetSphereGeometry().SetRadius(radius)
            particle.AddAsset(sphere_asset)
            particle.SetCollide(True)

            
            x = random.uniform(-1, 1)
            y = random.uniform(2, 4)  
            z = random.uniform(-1, 1)
            particle.SetPos(chrono.ChVectorD(x, y, z))

            
            vx = random.uniform(-1, 1)
            vy = random.uniform(-1, 1)
            vz = random.uniform(-1, 1)
            particle.SetWVel(chrono.ChVectorD(vx, vy, vz))

            
            particle.SetRot(chrono.ChQuaternionD(random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360)))

            
            system.Add(particle)

            
            attractor = CustomGravityForce(particle, system)
            particle.AddForce(attractor)


class CustomGravityForce(chrono.ChForce):
    def __init__(self, body, system):
        chrono.ChForce.__init__(self, "CustomGravity")
        self.body = body
        self.system = system

    def Setup(self):
        pass

    def DoStep(self, step):
        
        force_direction = -self.body.GetPos()
        force_direction.Normalize()
        force_magnitude = 1.0  
        force = force_direction * force_magnitude
        self.body.ApplyForce(force)



emitter_body = chrono.ChBodyEasy()
emitter_body.SetBodyFixed(True)
emitter_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(emitter_body)


emitter = ParticleEmitter(emitter_body, rate=20, min_radius=0.1, max_radius=0.2)
emitter_body.AddBehavior(emitter)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravity Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 3, -5))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)