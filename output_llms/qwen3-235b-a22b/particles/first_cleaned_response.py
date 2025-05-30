import pychrono as chrono
import pychrono.irrlicht as irrlicht
import random
import math


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


vis = irrlicht.ChVisualSystemIrrlicht()
sys.SetVisualSystem(vis)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravitational Attraction')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))

def create_particle():
    
    pos = chrono.ChVectorD(
        random.uniform(-2, 2),
        random.uniform(-2, 2),
        random.uniform(-2, 2)
    )

    
    vel = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )

    
    axis = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    ).GetNormalized()
    angle = random.uniform(0, 2 * math.pi)
    rot = chrono.ChQuaternionD()
    rot.Q_from_AngAxis(angle, axis)

    
    body = chrono.ChBody()
    body.SetPos(pos)
    body.SetPos_dt(vel)
    body.SetRot(rot)
    body.SetMass(1.0)  
    body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))  

    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.2)
    material.SetRestitution(0.5)
    body.SetMaterialSurface(material)

    
    shape_type = random.choice(['sphere', 'box'])
    if shape_type == 'sphere':
        radius = 0.1
        body.GetCollisionModel().AddSphere(radius)
        sphere = chrono.ChSphereShape()
        sphere.GetSphereGeometry().rad = radius
        body.AddAsset(sphere)
    else:
        box_size = chrono.ChVectorD(0.2, 0.2, 0.2)
        body.GetCollisionModel().AddBox(box_size.x, box_size.y, box_size.z)
        box = chrono.ChBoxShape()
        box.GetBoxGeometry().Size = box_size
        body.AddAsset(box)

    
    body.GetCollisionModel().Build()

    
    sys.Add(body)

    return body


for _ in range(5):
    create_particle()


timestep = 0.01
emission_interval = 1.0  
last_emission_time = 0.0


while vis.Run():
    current_time = sys.GetChTime()

    
    if current_time - last_emission_time >= emission_interval:
        create_particle()
        last_emission_time = current_time

    
    bodies = list(sys.Get_bodylist())
    for body_a in bodies:
        force_on_a = chrono.ChVectorD(0, 0, 0)
        for body_b in bodies:
            if body_a == body_b:
                continue
            r_vec = body_b.GetPos() - body_a.GetPos()
            r_mag = r_vec.Length()
            if r_mag < 0.01:  
                continue
            G = 1.0
            m_a = body_a.GetMass()
            m_b = body_b.GetMass()
            force_mag = G * m_a * m_b / (r_mag ** 2)
            force_dir = r_vec.GetNormalized()
            force_on_a += force_dir * force_mag
        body_a.SetF(force_on_a)  

    
    sys.DoStepDynamics(timestep)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()