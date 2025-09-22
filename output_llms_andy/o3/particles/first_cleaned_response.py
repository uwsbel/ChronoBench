import random, math, time
import pychrono as chrono

try:
    import pychrono.irrlicht as irr
except ImportError:
    raise RuntimeError("This script needs the Irrlicht module in your Chrono build.")




G_CONST          = 6.674e-2        
TIME_STEP        = 1.0/250.0
EMISSION_RATE    = 4.0             
MAX_PARTICLES    = 250             
EMIT_RADIUS      = 0.2             
INIT_VEL_MAG     = 0.4             
COLORS           = [ irr.SColorf(1,0,0), irr.SColorf(0,1,0),
                     irr.SColorf(0,0,1), irr.SColorf(1,1,0) ]




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
sys             = chrono.ChSystemSMC()           
sys.Set_G_acc(chrono.ChVectorD(0,0,0))          




app = irr.ChIrrApp(sys, "Particle Gravity Demo", irr.dimension2du(1280,720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddCamera(irr.vector3df(2,2,2), irr.vector3df(0,0,0))


floor = chrono.ChBodyEasyBox(4, 0.02, 4, 1000, True, True)
floor.SetBodyFixed(True)
sys.Add(floor)




particles   = []                   
time_buffer = 0.0                  

def random_vec_in_sphere(radius):
    while True:
        v = chrono.ChVectorD(random.uniform(-1,1),
                             random.uniform(-1,1),
                             random.uniform(-1,1))
        if v.Length2() <= 1:       
            return v * radius

def add_particle():
    shape_selector = random.choice(['sphere','box','cyl'])
    col = random.choice(COLORS)

    if shape_selector == 'sphere':
        r    = random.uniform(0.04,0.08)
        body = chrono.ChBodyEasySphere(r,          
                                       1000,       
                                       True, True) 
    elif shape_selector == 'box':
        sx,sy,sz  = [random.uniform(0.05,0.12) for _ in range(3)]
        body = chrono.ChBodyEasyBox(sx,sy,sz, 1000, True, True)
    else:                                  
        r  = random.uniform(0.04,0.08)
        h  = random.uniform(0.05,0.12)
        body = chrono.ChBodyEasyCylinder(r,h, 1000, True, True)

    
    pos = random_vec_in_sphere(EMIT_RADIUS) + chrono.ChVectorD(0,0.5,0)
    rot = chrono.ChQuaternionD()
    rot.Q_from_AngAxis(random.uniform(0,2*math.pi), chrono.ChVectorD(1,0,0))
    body.SetPos(pos)
    body.SetRot(rot)

    
    vdir = random_vec_in_sphere(1.0)
    if vdir.Length() > 0:
        vdir = vdir.GetNormalized()*random.uniform(0,INIT_VEL_MAG)
    body.SetPos_dt(vdir)

    
    for asset in body.GetAssets():
        if isinstance(asset, irr.ChColorAsset):
            asset.SetColor(col)

    sys.Add(body)
    particles.append(body)




def apply_mutual_gravity():
    n = len(particles)
    eps = 1e-9                       

    
    for p in particles:
        p.Empty_forces_accumulators()

    
    for i in range(n):
        bi = particles[i]
        mi = bi.GetMass()
        pi = bi.GetPos()

        for j in range(i+1, n):
            bj = particles[j]
            mj = bj.GetMass()
            pj = bj.GetPos()

            rij  = pj - pi
            d2   = rij.Length2() + eps
            invR = 1.0/math.sqrt(d2)
            Fmag = G_CONST * mi * mj * invR*invR    
            force_ij = rij * (Fmag*invR)            

            
            bi.Accumulate_force(force_ij , pi, False)
            bj.Accumulate_force(-force_ij, pj, False)




print("Simulation running...  ESC to quit.")
app.SetTimestep(TIME_STEP)

while app.GetDevice().run():
    
    if len(particles) < MAX_PARTICLES:
        time_buffer += TIME_STEP
        while time_buffer >= 1.0/EMISSION_RATE and len(particles) < MAX_PARTICLES:
            add_particle()
            time_buffer -= 1.0/EMISSION_RATE

    
    apply_mutual_gravity()
    sys.DoStepDynamics(TIME_STEP)

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    if app.GetDevice().isWindowActive() == False:
        time.sleep(0.002)