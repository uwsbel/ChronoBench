import random, math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# -----------------------------------------------------------------------------
# Utility: spawn a single random particle (sphere, box or cylinder)
# -----------------------------------------------------------------------------
def spawn_particle(system, app, density=1000):
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    # choose random size parameters
    if shape_type == 'sphere':
        r = random.uniform(0.1, 0.3)
        body = chrono.ChBodyEasySphere(r, density, True, True)
    elif shape_type == 'box':
        sx = random.uniform(0.1, 0.4)
        sy = random.uniform(0.1, 0.4)
        sz = random.uniform(0.1, 0.4)
        body = chrono.ChBodyEasyBox(sx, sy, sz, density, True, True)
    else:  # cylinder
        r = random.uniform(0.1, 0.3)
        h = random.uniform(0.1, 0.5)
        body = chrono.ChBodyEasyCylinder(r, h, density, True, True)

    # random position within a cube of side 4 centered at origin, but up high
    pos = chrono.ChVectorD(random.uniform(-2,2),
                           random.uniform( 1,5),
                           random.uniform(-2,2))
    body.SetPos(pos)

    # random initial linear velocity
    v = chrono.ChVectorD(random.uniform(-1,1),
                         random.uniform(-1,1),
                         random.uniform(-1,1))
    body.SetPos_dt(v)

    # random orientation
    axis = chrono.ChVectorD(random.random(), random.random(), random.random())
    axis.Normalize()
    angle = random.uniform(0, math.pi)
    q = chrono.Q_from_AngAxis(angle, axis)
    body.SetRot(q)

    # random angular velocity
    w = chrono.ChVectorD(random.uniform(-5,5),
                         random.uniform(-5,5),
                         random.uniform(-5,5))
    body.SetWvel_par(w)

    # add to the system
    system.Add(body)
    # bind and update the visualization assets for this new body
    app.AssetBind(body, True)
    app.AssetUpdate(body, True)
    return body

# -----------------------------------------------------------------------------
# Apply Newtonian attraction to all pairs of dynamic bodies in 'system'
# -----------------------------------------------------------------------------
def apply_gravity(system, G=1e-1, min_dist=0.1):
    bodies = list(system.Get_bodylist())
    n = len(bodies)
    # zeroing of force accumulators is done by Chrono at the start of each DoStep
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
                # gravitational magnitude
                F = G * mi * mj / (dist * dist)
                dir = diff / dist
                f_vec = dir * F
                # apply equal and opposite forces at centers
                bi.Accumulate_force(f_vec, chrono.ChVectorD(0,0,0))
                bj.Accumulate_force(-f_vec, chrono.ChVectorD(0,0,0))

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
# 1) Create the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0,0,0))  
# we turn off the default gravity since we implement pairwise gravity manually

# (optional) tuning solver
system.SetMaxItersSolverSpeed(100)
system.SetMaxItersSolverStab(100)

# 2) Create the Irrlicht application for real‐time visualization
app = chronoirr.ChIrrApp(system,
                         "Particle Gravitation Demo",
                         chronoirr.dimension2du(1024,768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(0,5,15))

# must bind all existing assets (none yet)
app.AssetBindAll()
app.AssetUpdateAll()

# 3) Simulation parameters
timestep     = 0.01
sim_time     = 0.0
end_time     = 50.0
spawn_rate   = 2.0        # particles per second
spawn_period = 1.0/spawn_rate
next_spawn   = 0.0

# 4) Main loop
app.SetTimestep(timestep)
while app.Run() and sim_time < end_time:
    app.BeginScene()
    app.DrawAll()

    # spawn new particles at the desired rate
    if sim_time >= next_spawn:
        spawn_particle(system, app)
        next_spawn += spawn_period

    # apply custom Newtonian forces
    apply_gravity(system, G=1e-1)

    # advance the simulation
    system.DoStepDynamics(timestep)

    app.EndScene()
    sim_time += timestep