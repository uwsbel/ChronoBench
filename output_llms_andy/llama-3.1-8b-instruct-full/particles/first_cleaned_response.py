import pychrono as chrono
    import pychrono.irrlicht as chronoirr
    import math
    import random
    import numpy as np

    
    R = 0.4  
    M = 100.0  
    NPARTICLES = 20  
    EMITTER_LIFETIME = 1000.0  
    GRAVITY = False  

    
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    ground = chrono.ChBody()
    sys.Add(ground)
    ground.SetFixed(True)  
    ground.EnableCollision(False)  

    
    emitter = chrono.ChParticleEmitter()
    emitter.SetParticlesPerSecond(NPARTICLES)
    emitter.SetUseParticleReservoir(True)
    emitter.SetParticleReservoirAmount(200)
    emitter.SetRandomDirection(True)
    emitter.SetRandomPosition(True)
    emitter.SetPosition(chrono.ChVector3d(0, 0, 0))
    emitter.SetVelocityFunction(chrono.ChFunction_Random(0, 0.2))
    emitter.SetSpinFunction(chrono.ChFunction_Random(0, 2 * chrono.CH_PI))
    sys.AddEmitter(emitter)

    
    def custom_gravitational_force(p1, p2):
        F = chrono.ChVector3d(0, 0, 0)
        d = p2.position - p1.position
        r2 = d.Length2()
        if r2 > 0:
            r = math.sqrt(r2)
            F = (M * m / (r2 * r2)) * d
        return F

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Particle emitter demo')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 3, 6))
    vis.AddTypicalLights()

    
    t = 0
    while vis.Run():
        emitter.SetParticleReservoirAmount(200 - int(NPARTICLES * (t / EMITTER_LIFETIME)))
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(1e-2)
        t += 1e-2