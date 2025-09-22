import pychrono as ch
    import pychrono.irrlicht as chrig
    import numpy as np
    import math

    
    g = 9.81  
    m = 1.0   
    r = 0.1   
    L = 10.0  
    T = 30.0  

    
    n_particles = 1000

    
    time_step = 1e-3

    
    G = 0.1

    
    emitter = ch.ChParticleEmitter()

    
    container = ch.ChParticleContainer()

    
    vis = ch.ChVisualizeParticles()

    
    sys = ch.ChInteractiveSystemIrrlicht()
    sys.AttachVisualizeParticles(vis)
    sys.SetWindowSize(1024, 768)
    sys.SetWindowTitle('Particle Emitter Demo')
    sys.Initialize()
    sys.AddLogo(ch.ChLogoType.NONE)
    sys.SetChaseCamera(6.0)

    
    center = ch.ChVector3d(0, 0, 0)

    
    material = ch.ChContactMaterialNSC()

    
    creator = ch.ChRandomCreator()

    
    msys = ch.ChSystemNSC()
    msys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -g))
    msys.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
    msys.SetSolverType(ch.ChSolver.Type_SOR)

    
    sphere_creator = ch.ChCreatorSphere()

    
    box_creator = ch.ChCreatorBox()

    
    cylinder_creator = ch.ChCreatorCylinder()

    
    particle_shape_creator = ch.ChCreatorParticleShape()

    
    def emitter_update(emitter, time):
        
        particle_shape = particle_shape_creator.Create()

        
        particle_shape.SetRadius(r)
        particle_shape.SetMass(m)
        particle_shape.SetInertiaTensor(0.4, 0.4, 0.4)

        
        particle = ch.ChParticle()
        particle.SetPos(ch.ChVector3d(0, 0, 0))
        particle.SetPos(particle_shape.GetPos())
        particle.SetRot(ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 0, 1)))
        particle.SetMass(m)
        particle.SetInertiaTensor(0.4, 0.4, 0.4)

        
        container.AddParticle(particle)

        
        pos = ch.ChVector3d(
            creator.GetRandUniform(-L, L),
            creator.GetRandUniform(-L, L),
            creator.GetRandUniform(-L, L)
        )
        particle.SetPos(pos)
        particle.SetPos(particle_shape.GetPos())

        
        particle.SetPos(
            ch.ChVector3d(
                creator.GetRandUniform(-10, 10),
                creator.GetRandUniform(-10, 10),
                creator.GetRandUniform(-10, 10)
            )
        )

        
        msys.Add(particle)

        
        sphere = sphere_creator.Create()
        sphere.SetPos(ch.ChVector3d(0, 0, 0))
        sphere.SetPos(sphere_shape.GetPos())

        
        box = box_creator.Create()
        box.SetPos(ch.ChVector3d(0, 0, 0))
        box.SetPos(box_shape.GetPos())

        
        cylinder = cylinder_creator.Create()
        cylinder.SetPos(ch.ChVector3d(0, 0, 0))
        cylinder.SetPos(cylinder_shape.GetPos())

        
        particle_shape = particle_shape_creator.Create()
        particle_shape.SetPos(ch.ChVector3d(0, 0, 0))
        particle_shape.SetPos(particle_shape.GetPos())

        
        particle = ch.ChParticle()
        particle.SetPos(ch.ChVector3d(0, 0, 0))
        particle.SetPos(particle_shape.GetPos())
        particle.SetRot(ch.QuatFromAngleAxis(0, ch.Ch
print("error happened with only start ```python")