import random, math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr




def spawn_particle(system, app, density=1000):
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    
    if shape_type == 'sphere':
        r = random.uniform(0.1, 0.3)
        body = chrono.ChBodyEasySphere(r, density, True, True)
    elif shape_type == 'box':
        sx = random.uniform(0.1, 0.4)
        sy = random.uniform(0.1, 0.4)
        sz = random.uniform(0.1, 0.4)
        body = chrono.ChBodyEasyBox(sx, sy, sz, density, True, True)
    else:  
        r = random.uniform(0.1, 0.3)
        h = random.uniform(0.1, 0.5)
        body = chrono.ChBodyEasyCylinder(r, h, density, True, True)

    
    pos = chrono.ChVectorD(random.uniform(-2,2),
                           random.uniform( 1,5),
                           random.uniform(-2,2))
    body.SetPos(pos)

    
    v = chrono.ChVectorD(random.uniform(-1,1),
                         random.uniform(-1,1),
                         random.uniform(-1,1))
    body.SetPos_dt(v)

    
    axis = chrono.ChVectorD(random.random(), random.random(), random.random())
    axis.Normalize()
    angle = random.uniform(0, math.pi)
    q = chrono.Q_from_AngAxis(angle, axis)
    body.SetRot(q)

    
    w = chrono.ChVectorD(random.uniform(-5,5),
                         random.uniform(-5,5),
                         random.uniform(-5,5))
    body.SetWvel_par(w)

    
    system.Add(body)
    
    app.AssetBind(body, True)
    app.AssetUpdate(body, True)
    return body




def apply_gravity(system, G=1e-1, min_dist=0.1):
    bodies = list(system.Get_bodylist())
    n = len(bodies)
    
    for i in range(n):
        bi = bodies[i]
        if bi.IsDynamical():
            mi = bi.GetMass()
            pi = bi.GetPos()
            for j in range(i+1, n):
                bj = bodies[j]
                if not bj.IsDynamical():
                    continue
                mj = bj.GetMass()
                pj = bj.GetPos()
                diff = pj - pi
                dist = diff.Length()
                if dist < min_dist:
                    continue
                
                F = G * mi * mj / (dist * dist)
                dir = diff / dist
                f_vec = dir * F
                
                bi.Accumulate_force(f_vec, chrono.ChVectorD(0,0,0))
                bj.Accumulate_force(-f_vec, chrono.ChVectorD(0,0,0))





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0,0,0))  



system.SetMaxItersSolverSpeed(100)
system.SetMaxItersSolverStab(100)


app = chronoirr.ChIrrApp(system,
                         "Particle Gravitation Demo",
                         chronoirr.dimension2du(1024,768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(0,5,15))


app.AssetBindAll()
app.AssetUpdateAll()


timestep     = 0.01
sim_time     = 0.0
end_time     = 50.0
spawn_rate   = 2.0        
spawn_period = 1.0/spawn_rate
next_spawn   = 0.0


app.SetTimestep(timestep)
while app.Run() and sim_time < end_time:
    app.BeginScene()
    app.DrawAll()

    
    if sim_time >= next_spawn:
        spawn_particle(system, app)
        next_spawn += spawn_period

    
    apply_gravity(system, G=1e-1)

    
    system.DoStepDynamics(timestep)

    app.EndScene()
    sim_time += timestep