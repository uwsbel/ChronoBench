import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import random


chrono.SetChronoDataPath('path/to/chrono/data')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1024, 768)
visualization.SetWindowTitle('Gravitational Attraction Simulation')
visualization.Initialize()
visualization.AddTypicalSky()
visualization.AddTypicalLogo()
visualization.AddLightWithShadow(chrono.ChCoordsysD(chrono.ChVectorD(2, 2, 5)), chrono.ChVectorD(1, 1, -1), 50, 50, 10, 50, 50)


def create_random_particle(system):
    mass = random.uniform(0.1, 1.0)
    radius = random.uniform(0.05, 0.1)

    
    sphere_shape = chrono.ChSphereShape()
    sphere_shape.GetSphereGeometry().rad = radius

    
    body = chrono.ChBodyEasySphere(mass, radius, material=chrono.ChMaterialSurfaceNSC())
    body.SetPos(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(0, 2), random.uniform(-1, 1)))
    body.SetRot(chrono.ChQuaternionD(1, 0, 0, random.uniform(0, 2 * 3.141592653589793)))
    body.SetVel(chrono.ChVectorD(random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5)))
    body.SetAngVel(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))

    
    system.Add(body)

    
    visualization.Add(body, sphere_shape)


def emit_particles(system, num_particles):
    for _ in range(num_particles):
        create_random_particle(system)


emit_particles(system, 50)


def gravitational_force(body1, body2, G=6.67430e-11):
    r = body2.GetPos() - body1.GetPos()
    distance = r.Length()
    if distance == 0:
        return chrono.ChVectorD(0, 0, 0)

    force_magnitude = G * (body1.GetMass() * body2.GetMass()) / (distance ** 2)
    force_direction = r.Normalize()
    force = force_direction * force_magnitude
    return force


def apply_gravitational_forces(system):
    bodies = system.Get_bodylist()
    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            body1 = bodies[i]
            body2 = bodies[j]
            force = gravitational_force(body1, body2)
            body1.AddForce(force)
            body2.AddForce(-force)


def run_simulation(system, visualization):
    while visualization.Run():
        system.DoStepDynamics(0.01)  
        apply_gravitational_forces(system)
        visualization.Render()


run_simulation(system, visualization)