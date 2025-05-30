import pychrono as chrono
import pychronoirrlicht as chronoirr
import random
import math


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, 0))


application = chronoirr.ChIrrApp(system, "Gravitational Particle Attraction", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 5, -20))
application.AddLightWithShadow(chronoirr.vector3df(0, 10, -10), chronoirr.vector3df(0, 0, 0), 9, 4, 20, 40, 512)

application.AssetBindAll()
application.AssetUpdateAll()


class ParticleEmitter:
    def __init__(self, system, spawn_rate=10):
        self.system = system
        self.spawn_rate = spawn_rate  
        self.time_accum = 0.0
        self.spawn_interval = 1.0 / spawn_rate
        self.particles = []

    def random_shape(self):
        
        shape_type = random.choice(['sphere', 'box', 'cylinder'])
        size = random.uniform(0.1, 0.4)  

        if shape_type == 'sphere':
            
            shape = chrono.ChBodyEasySphere(size, 1000, True, True)
        elif shape_type == 'box':
            dims = chrono.ChVectorD(size * random.uniform(0.5, 1.5), size * random.uniform(0.5, 1.5), size * random.uniform(0.5, 1.5))
            shape = chrono.ChBodyEasyBox(dims.x, dims.y, dims.z, 1000, True, True)
        else:
            radius = size * random.uniform(0.5, 1.0)
            length = size * random.uniform(0.5, 2.0)
            shape = chrono.ChBodyEasyCylinder(radius, length, 1000, True, True)

        return shape

    def random_position(self):
        
        x = random.uniform(-5, 5)
        y = random.uniform(0.5, 5)
        z = random.uniform(-5, 5)
        return chrono.ChVectorD(x, y, z)

    def random_velocity(self):
        
        vx = random.uniform(-1.0, 1.0)
        vy = random.uniform(-1.0, 1.0)
        vz = random.uniform(-1.0, 1.0)
        return chrono.ChVectorD(vx * 2, vy * 2, vz * 2)

    def random_orientation(self):
        
        angle = random.uniform(0, 2*math.pi)
        axis = chrono.ChVectorD(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1))
        axis.Normalize()
        q = chrono.ChQuaternionD()
        q.Q_from_AngAxis(angle, axis)
        return q

    def emit(self, time_step):
        
        self.time_accum += time_step
        while self.time_accum > self.spawn_interval:
            self.time_accum -= self.spawn_interval
            particle = self.random_shape()
            particle.SetPos(self.random_position())
            particle.SetRot(self.random_orientation())
            particle.SetPos_dt(self.random_velocity())
            particle.SetMass(1.0)  
            particle.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
            particle.SetCollide(True)

            
            particle.GetCollisionModel().ClearModel()
            if isinstance(particle, chrono.ChBodyEasySphere):
                particle.GetCollisionModel().AddSphere(particle.GetRadius(), chrono.ChVectorD(0,0,0))
            elif isinstance(particle, chrono.ChBodyEasyBox):
                dims = particle.GetBoxGeometry().Size
                particle.GetCollisionModel().AddBox(dims.x*0.5, dims.y*0.5, dims.z*0.5)
            elif isinstance(particle, chrono.ChBodyEasyCylinder):
                radius = particle.GetCylinderGeometry().rad
                length = particle.GetCylinderGeometry().p2.z - particle.GetCylinderGeometry().p1.z
                particle.GetCollisionModel().AddCylinder(radius, radius, length*0.5)
            particle.GetCollisionModel().BuildModel()

            self.system.Add(particle)
            self.particles.append(particle)

    def get_particles(self):
        return self.particles





G_const = 5e-3

def apply_gravitational_forces(particles):
    n = len(particles)
    
    for p in particles:
        p.Empty_forces_accumulators()

    
    for i in range(n):
        pi = particles[i]
        pos_i = pi.GetPos()
        mi = pi.GetMass()
        for j in range(i + 1, n):
            pj = particles[j]
            pos_j = pj.GetPos()
            mj = pj.GetMass()

            r_vec = pos_j - pos_i
            dist2 = r_vec.Length2()

            if dist2 < 1e-6:
                continue  

            dist = math.sqrt(dist2)
            force_mag = G_const * mi * mj / dist2
            force_dir = r_vec / dist  

            force = force_dir * force_mag

            
            pi.Empty_forces_accumulators()
            pi.Accumulate_force(force, pos_i, False)

            pj.Accumulate_force(-force, pos_j, False)


emitter = ParticleEmitter(system, spawn_rate=20)


time_step = 0.005

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    emitter.emit(time_step)

    
    particles = emitter.get_particles()
    if len(particles) > 1:
        apply_gravitational_forces(particles)

    system.DoStepDynamics(time_step)
    application.EndScene()