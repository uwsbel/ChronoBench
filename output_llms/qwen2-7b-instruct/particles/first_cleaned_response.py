import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.Ch纪元().Initialize()


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChIrrApp(system, "PyChrono Particle Attraction Simulation", chronoirr.dimension2du(800, 600))
vis.AddTypicalCamera(chrono.ChVectorD(0, 10, 10))


num_particles = 100
particle_radius = 0.1
particle_mass = 1.0
particle_density = 1000.0

for i in range(num_particles):
    
    pos = chrono.ChVectorD(chrono.GetRandom(-10.0, 10.0), chrono.GetRandom(-10.0, 10.0), chrono.GetRandom(-10.0, 10.0))
    
    vel = chrono.ChVectorD(chrono.GetRandom(-5.0, 5.0), chrono.GetRandom(-5.0, 5.0), chrono.GetRandom(-5.0, 5.0))
    
    rot = chrono.ChQuaternionD()
    rot.Rotation(chrono.ChVectorD(chrono.GetRandom(-1.0, 1.0), chrono.GetRandom(-1.0, 1.0), chrono.GetRandom(-1.0, 1.0)), chrono.ChVectorD(0, 0, 1))
    
    
    sphere_shape = chrono.ChSphereShape()
    sphere_shape.GetSphere().rad = particle_radius
    
    
    sphere_body = chrono.ChBody()
    sphere_body.SetIdentifier(i)
    sphere_body.SetBodyFixed(False)
    sphere_body.SetPos(pos)
    sphere_body.SetRot(rot)
    sphere_body.SetMass(chrono.ChMassProperties(mass=particle_mass, inertia=chrono.ChVectorD(1.0, 1.0, 1.0)))
    sphere_body.AddAsset(chrono.ChTexture("particle.png"))
    sphere_body.GetCollisionModel().ClearModel()
    sphere_body.GetCollisionModel().AddSphere(particle_radius)
    sphere_body.GetCollisionModel().BuildModel()
    sphere_body.SetCollide(True)
    sphere_body.GetVisualShapeFactory().SetTextureScale(chrono.ChVectorD(1.0, 1.0, 1.0))
    
    
    system.Add(sphere_body)
    system.Add(sphere_shape)
    
    
    sphere_body.AddForce(chrono.ChVectorD(0, -particle_density * particle_mass * 9.81, 0), chrono.ChFrameD(pos, rot))


vis.AddTypicalLights()
vis.AddTypicalSky()
vis.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddTypicalCamera(chrono.ChVectorD(0, 10, 10))
vis.SetTimestep(0.001)
vis.Start()
vis.Run()


chrono.Ch纪元().Finalize()