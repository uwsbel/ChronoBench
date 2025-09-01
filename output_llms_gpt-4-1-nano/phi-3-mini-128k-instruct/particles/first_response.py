import pychrono as chrono

import pychrono.openvr as vr

import pychrono.visual as vis

import pychrono.analytic_motion as am

import pychrono.analytic_force as af

import numpy as np

import random

import irrlicht as ir

import irrlicht.keyboard as keyboard


# Initialize PyChrono

chrono.ChSystemContext().SetGravity(chrono.ChVectorD(0, -9.81, 0))


# Create a particle emitter

particle_emitter = chrono.ChParticleEmitter()

particle_emitter.SetShape(chrono.ChSphereShape(1.0))

particle_emitter.SetPosition(chrono.ChVectorD(0, 100, 0))

particle_emitter.SetVelocity(chrono.ChVectorD(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)))

particle_emitter.SetOrientation(chrono.ChQuaternionD(random.uniform(0, 2*np.pi), random.uniform(0, np.pi), random.uniform(0, 2*np.pi)))


# Add the particle emitter to the system

system = chrono.ChSystemContext()

system.AddParticleEmitter(particle_emitter)


# Add a custom gravitational force to simulate attraction

def custom_gravitational_force(particle, system):

    for other_particle in system.GetParticles():

        if particle is not other_particle:

            distance = particle.GetPosition() - other_particle.GetPosition()

            force_magnitude = chrono.ChVectorD(distance.x, distance.y, distance.z) * chrono.ChVectorD(0, -1, 0).Length()

            force = force_magnitude * chrono.ChVectorD(0, 1, 0)

            particle.AddForce(force)

            other_particle.AddForce(-force)


# Set up the visualization with Irrlicht

scene = vis.ChVisualScene()

scene.AddActor(vis.ChActor(system.GetParticles()[0].GetShape()))


# Set up the Irrlicht application

application = vr.ChApplicationVR_Full(chrono.ChSystemContext())

application.SetWindowTitle("Particle Simulation")

application.SetWindowSize(1024, 768)

application.AddScene(scene)

application.AddTimer(1/60.0)


# Main loop

while application.GetDevice().IsOpened():

    application.BeginScene()

    application.DrawAll()

    application.EndScene()

    application.Update()


    # Update the custom gravitational forces

    for particle in system.GetParticles():

        custom_gravitational_force(particle, system)


    # Handle user input

    keyboard.wait_for_key(application)

    if keyboard.is_pressed('q'):

        application.Close()