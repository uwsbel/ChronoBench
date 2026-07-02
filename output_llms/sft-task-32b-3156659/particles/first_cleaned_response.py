import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

system = chrono.ChSystemNSC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 720)
vis.SetWindowTitle('Gravitational Particle Attraction')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chronoirr.ChVector3d(0, 3, -10), chronoirr.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

particles = []

def create_particle():
    if np.random.rand() < 0.5:
        radius = np.random.uniform(0.05, 0.15)
        body = chrono.ChBodyEasySphere(radius, 1000, True, True)
    else:
        size = np.random.uniform(0.1, 0.3)
        body = chrono.ChBodyEasyBox(size, size, size, 1000, True, True)
    pos = chrono.ChVector3d(
        np.random.uniform(-2, 2),
        np.random.uniform(1, 3),
        np.random.uniform(-2, 2)
    )
    body.SetPos(pos)
    vel = chrono.ChVector3d(
        np.random.uniform(-0.5, 0.5),
        np.random.uniform(-0.5, 0.5),
        np.random.uniform(-0.5, 0.5)
    )
    body.SetPosDt(vel)
    angle = np.random.uniform(0, 2 * np.pi)
    axis = chrono.ChVector3d(np.random.rand(), np.random.rand(), np.random.rand()).GetNormalized()
    q = chrono.ChQuaterniond()
    q.Q_from_AngAxis(angle, axis)
    body.SetRot(q)
    color = chrono.ChColor(np.random.rand(), np.random.rand(), np.random.rand())
    body.GetVisualShape(0).SetColor(color)
    system.Add(body)
    particles.append(body)

time_since_last_emit = 0.0
emit_interval = 0.5
time_step = 0.01

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    G = 50.0
    num_particles = len(particles)
    for i in range(num_particles):
        for j in range(i + 1, num_particles):
            body_i = particles[i]
            body_j = particles[j]
            pos_i = body_i.GetPos()
            pos_j = body_j.GetPos()
            r_vec = chrono.ChVector3d(pos_j.x - pos_i.x, pos_j.y - pos_i.y, pos_j.z - pos_i.z)
            r_mag = r_vec.Length()
            if r_mag < 0.1:
                continue
            r_hat = r_vec / r_mag
            mass_i = body_i.GetMass()
            mass_j = body_j.GetMass()
            force_mag = G * mass_i * mass_j / (r_mag ** 2)
            force_vec = r_hat * force_mag
            body_i.ApplyForce(force_vec, pos_i)
            body_j.ApplyForce(-force_vec, pos_j)
    
    
    current_time = system.GetChTime()
    if current_time - time_since_last_emit > emit_interval:
        create_particle()
        time_since_last_emit = current_time
    
    system.DoStepDynamics(time_step)