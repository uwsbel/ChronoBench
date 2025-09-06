import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


def emit_particle(sys):
    
    angle = np.random.uniform(0, 2*np.pi)
    radius = np.random.uniform(0, 2)
    x = radius * np.cos(angle)
    z = radius * np.sin(angle)
    y = 5  
    pos = chrono.ChVector3d(x, y, z)
    
    
    speed = np.random.uniform(0.1, 0.5)
    vel_dir = chrono.ChVector3d(np.random.uniform(-1,1), np.random.uniform(-1,1), np.random.uniform(-1,1))
    vel_dir.Normalize()
    vel = vel_dir * speed
    
    
    if np.random.rand() > 0.5:
        radius = np.random.uniform(0.1, 0.3)
        body = chrono.ChBodyEasySphere(radius, 1000, True, True)
    else:
        size = chrono.ChVector3d(np.random.uniform(0.2, 0.5), np.random.uniform(0.2, 0.5), np.random.uniform(0.2, 0.5))
        body = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, True)
    
    
    body.SetPos(pos)
    body.SetPos_dt(vel)
    angle_rot = np.random.uniform(0, 2*np.pi)
    axis = chrono.ChVector3d(np.random.rand(), np.random.rand(), np.random.rand())
    axis.Normalize()
    quat = chrono.ChQuaterniond()
    quat.Q_from_AngAxis(angle_rot, axis)
    body.SetRot(quat)
    
    
    color = chrono.ChColor(np.random.rand(), np.random.rand(), np.random.rand())
    body.GetVisualShape(0).SetColor(color)
    
    
    sys.Add(body)


sys = chrono.ChSystemNSC()
sys.Set_G_value(chrono.ChVector3d(0, 0, 0))  
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


material = chrono.ChContactMaterialNSC()
material.SetFriction(0.2)
sys.SetDefaultMaterial(material)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Particle Gravity Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 3, 5))
vis.AddTypicalLights()


step = 0
while vis.Run():
    
    if step % 50 == 0:
        emit_particle(sys)
    
    
    G = 1e-3  
    for body in sys.Get_bodylist():
        total_force = chrono.ChVector3d(0, 0, 0)
        for other in sys.Get_bodylist():
            if body == other:
                continue
            r_vec = other.GetPos() - body.GetPos()
            distance = r_vec.Length()
            if distance < 1e-3:  
                continue
            direction = r_vec.GetNormalized()
            F = G * body.GetMass() * other.GetMass() / (distance ** 2)
            total_force += F * direction
        
        body.Empty_forces()
        body.AddForce(total_force)
    
    
    sys.DoStepDynamics(0.01)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    step += 1