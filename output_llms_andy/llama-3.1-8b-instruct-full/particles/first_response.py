import pychrono as chrono
    import pychrono.irrlicht as chronoirr
    import math
    import random
    import numpy as np

    # Define constants and parameters
    R = 0.4  # Particle radius
    M = 100.0  # Particle mass
    NPARTICLES = 20  # Number of particles
    EMITTER_LIFETIME = 1000.0  # Emitter lifetime
    GRAVITY = False  # Flag for gravity

    # Create Chrono physical system
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create ground body and add it to the system
    ground = chrono.ChBody()
    sys.Add(ground)
    ground.SetFixed(True)  # Fix the ground
    ground.EnableCollision(False)  # Disable collision on ground

    # Create and initialize particle emitter
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

    # Define a custom gravitational force function
    def custom_gravitational_force(p1, p2):
        F = chrono.ChVector3d(0, 0, 0)
        d = p2.position - p1.position
        r2 = d.Length2()
        if r2 > 0:
            r = math.sqrt(r2)
            F = (M * m / (r2 * r2)) * d
        return F

    # Create Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Particle emitter demo')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 3, 6))
    vis.AddTypicalLights()

    # Simulation loop
    t = 0
    while vis.Run():
        emitter.SetParticleReservoirAmount(200 - int(NPARTICLES * (t / EMITTER_LIFETIME)))
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(1e-2)
        t += 1e-2