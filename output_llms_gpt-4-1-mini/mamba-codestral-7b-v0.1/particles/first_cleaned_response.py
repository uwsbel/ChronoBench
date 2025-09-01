import pychrono


system = pychrono.ChSystemSMC()



particle_system = pychrono.ChParticleSystem()
system.AddSystem(particle_system)



system.Set_G_acc(pychrono.ChVectorD(0, 0, -9.81))



def particle_emitter():
    
    particle = pychrono.ChParticle()
    particle.SetPos(pychrono.ChVectorD(...))  
    particle.SetVel(pychrono.ChVectorD(...))  
    particle.SetRot(pychrono.ChQuaternionD(...))  
    particle.SetShape(pychrono.ChShapeSphere(...))  
    particle_system.AddParticle(particle)



def visualize_simulation():
    
    ...

    
    ...

    
    def custom_gravitational_force(particle):
        
        ...
        particle.SetForce(force)

    
    while system.GetChTime() < simulation_time:
        
        system.DoStepDynamics(stepsize)

        
        for particle in particle_system.GetParticlesList():
            custom_gravitational_force(particle)

        
        ...


visualize_simulation()