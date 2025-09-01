import pychrono as chrono
import pychrono.irrlicht as irrlicht
import random
import math

# Custom gravitational force calculator
class GravitationalForce(chrono.ChPhysicsItem):
    def __init__(self, G):
        super().__init__()
        self.G = G
        self.bodies = []

    def AddBody(self, body):
        self.bodies.append(body)

    def Update(self, time, update_assets=True):
        n = len(self.bodies)
        for i in range(n):
            body_i = self.bodies[i]
            pos_i = body_i.GetPos()
            mass_i = body_i.GetMass()
            for j in range(i+1, n):
                body_j = self.bodies[j]
                pos_j = body_j.GetPos()
                mass_j = body_j.GetMass()
                delta = pos_j - pos_i
                r = delta.Length()
                if r < 1e-6:
                    continue
                force_magnitude = self.G * mass_i * mass_j / (r**2)
                force_dir = delta / r
                force = force_dir * force_magnitude
                body_i.AddForce(force)
                body_j.AddForce(-force)

# Particle emitter class
class ParticleEmitter:
    def __init__(self, system, grav_force, position):
        self.system = system
        self.grav_force = grav_force
        self.position = position
        self.time_since_last_emission = 0
        self.density = 500  # kg/m³

    def emit_particle(self):
        # Random shape selection
        shape_type = random.choice(['sphere', 'box'])
        
        # Common body properties
        body = chrono.ChBody()
        body.SetCollide(False)
        
        # Shape-specific properties
        if shape_type == 'sphere':
            radius = random.uniform(0.2, 0.4)
            mass = (4/3) * math.pi * (radius**3) * self.density
            body.SetMass(mass)
            body.SetInertiaXX(chrono.ChVectorD(0.4*mass*radius**2, 0.4*mass*radius**2, 0.4*mass*radius**2))
            vis_shape = chrono.ChSphereShape(radius)
        else:
            size = chrono.ChVectorD(random.uniform(0.2, 0.4), 
                                  random.uniform(0.2, 0.4), 
                                  random.uniform(0.2, 0.4))
            mass = size.x * size.y * size.z * 8 * self.density
            body.SetMass(mass)
            body.SetInertiaXX(chrono.ChVectorD(mass*(size.y**2 + size.z**2)/3,
                                             mass*(size.x**2 + size.z**2)/3,
                                             mass*(size.x**2 + size.y**2)/3))
            vis_shape = chrono.ChBoxShape(size*2)
        
        # Visual properties
        vis_shape.SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
        body.AddVisualShape(vis_shape)
        
        # Initial position and orientation
        pos = self.position + chrono.ChVectorD(random.uniform(-2, 2),
                                             random.uniform(-2, 2),
                                             random.uniform(-2, 2))
        body.SetPos(pos)
        body.SetRot(chrono.Q_from_AngAxis(random.uniform(0, 2*math.pi), 
                                        chrono.ChVectorD(random.random(), 
                                                       random.random(), 
                                                       random.random()).GetNormalized()))
        
        # Initial velocity
        vel = chrono.ChVectorD(random.uniform(-0.5, 0.5),
                             random.uniform(-0.5, 0.5),
                             random.uniform(-0.5, 0.5))
        body.SetPos_dt(vel)
        
        # Add to system
        self.system.Add(body)
        self.grav_force.AddBody(body)

# Simulation setup
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable default gravity

# Create gravitational force system
grav_force = GravitationalForce(G=1e-2)  # Increased G for visible effects
system.Add(grav_force)

# Create particle emitter
emitter = ParticleEmitter(system, grav_force, chrono.ChVectorD(0, 0, 0))

# Visualization setup
vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gravitational Particle System')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0, 15), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Simulation loop parameters
time_step = 0.01
emission_interval = 0.2
current_time = 0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Emit new particles
    emitter.time_since_last_emission += time_step
    if emitter.time_since_last_emission >= emission_interval:
        emitter.emit_particle()
        emitter.time_since_last_emission = 0
    
    # Simulation step
    system.DoStepDynamics(time_step)
    current_time += time_step