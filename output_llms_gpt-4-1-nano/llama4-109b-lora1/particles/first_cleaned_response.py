import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.postprocess as post
import numpy as np
import random


chrono.SetChairsThreading(chrono.GetNumProcs())


system = chrono.ChSystemNSC()


system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


app = irr.ChIrrAppWrappers()
app.SetSystem(system)
app.SetRenderMode(irr.RenderMode_VITRUAL_REALITY)
app.SetWindowSize(800, 600)
app.SetWindowTitle("Gravitational Attraction between Particles")
app.ShowLogo()


emitter = chrono.ChParticleEmitter()
emitter.SetRadius(0.1)
emitter.SetInitialSpeed(10)
emitter.SetInitialSize(chrono.ChVector3d(0.1, 0.1, 0.1))
emitter.SetInitialShapeType(chrono.ChVisualShapeType_SPHERE)
emitter.SetInitialShapeColor(chrono.ChColor(1, 0, 0))
emitter.SetInitialMaterial(chrono.ChContactMaterialNSC())
emitter.SetInitialDensity(1000)
emitter.SetInitialMass(1)
emitter.SetInitialInertia(chrono.ChVector3d(1, 1, 1))
emitter.SetInitialAngularDamping(0.5)
emitter.SetInitialLinearDamping(0.5)


system.Add(emitter.GetEmitter())


system.SetFixedStep(0.01)


end_time = 10.0


def custom_gravity_force(particle):
    
    force = chrono.ChVector3d(0, 0, 0)
    for other_particle in system.GetChContactContainer().GetParticles():
        if other_particle != particle:
            distance = (particle.GetPos() - other_particle.GetPos()).Length()
            if distance > 0:
                force += (other_particle.GetPos() - particle.GetPos()) / distance * 0.1 / distance
    return force


def custom_gravity_force_callback(particle):
    particle.AddForce(custom_gravity_force(particle), chrono.ChFrame(particle.GetTime()))
    return


def visualization_callback():
    
    app.BeginScene()
    app.AddTypicalLights()
    app.AddCamera(chrono.ChVector3d(0, 0, 10), chrono.ChVector3d(0, 0, 0))
    app.EndScene()


while app.GetDevice().run() and (system.GetChTime() < end_time):
    
    system.DoStepDynamics(system.GetFixedStep())

    
    visualization_callback()


app.GetDevice().closeDevice()