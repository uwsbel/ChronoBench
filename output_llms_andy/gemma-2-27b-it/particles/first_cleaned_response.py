import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random







my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Particle Gravity Simulation")
vis.Initialize()
vis.AddTypicalSky()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))


emitter = chrono.ChParticleEmitter()
emitter.SetPos(chrono.ChVectorD(0, 10, 0))
emitter.SetFlowRate(10)  


emitter.SetParticleMass(0.1)
emitter.SetParticleRadius(0.2)


shapes = [chrono.ChSphereShape, chrono.ChCapsuleShape, chrono.ChBoxShape]






while vis.Run():
    
    for _ in range(emitter.GetFlowRate()):
        
        pos = chrono.ChVectorD(random.uniform(-5, 5), random.uniform(5, 15), random.uniform(-5, 5))
        vel = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
        rot = chrono.ChQuaternionD(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)).Normalize()

        
        particle = chrono.ChBody()
        particle.SetPos(pos)
        particle.SetBodyFixed(False)
        particle.SetMass(emitter.GetParticleMass())
        particle.SetCollide(True)
        particle.AddAsset(shapes[random.randint(0, len(shapes) - 1)]())
        particle.SetRot(rot)
        particle.SetLinVelocity(vel)

        
        my_system.AddBody(particle)

    
    for i in range(my_system.Get_bodylist().size()):
        for j in range(i + 1, my_system.Get_bodylist().size()):
            body1 = my_system.Get_bodylist()[i]
            body2 = my_system.Get_bodylist()[j]
            
            
            r = body2.GetPos() - body1.GetPos()
            
            
            force = (6.67430e-11 * body1.GetMass() * body2.GetMass() / r.Length2()) * r.GetNormalized()
            
            
            body1.AddForce(force)
            body2.AddForce(-force)

    
    my_system.DoStepDynamics(0.01)
    vis.Render()