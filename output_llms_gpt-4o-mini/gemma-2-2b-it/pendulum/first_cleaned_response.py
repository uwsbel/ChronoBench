import pychrono as chrono
import pychrono.irrlicht as irr
import numpy as np


chrono.init()


engine = irr.IrrlichtEngine()


world = chrono.World()


ground = chrono.Body(chrono.BodyType.GROUND)
ground.setMass(1000)
ground.setInertia(chrono.Inertia(1000, 0, 0))
ground.setPosition(chrono.Vec3(0, 0, 0))
world.addBody(ground)


pendulum = chrono.Body(chrono.BodyType.RIGID_BODY)
pendulum.setMass(1)
pendulum.setInertia(chrono.Inertia(0.1, 0, 0))
pendulum.setPosition(chrono.Vec3(0, 1, 0))
pendulum.setAngularVelocity(chrono.Vec3(0, 0, 0))


joint = chrono.RevoluteJoint(pendulum, ground)
world.addBody(pendulum)
world.addBody(ground)
world.addJoint(joint)


dt = 0.01
time_step = chrono.TimeStep(dt)


irr.setCamera(chrono.Vec3(0, 0, 10), chrono.Vec3(0, 0, 0), chrono.Vec3(0, 1, 0))


world.run(time_step)


while True:
    
    position = pendulum.getPosition()
    velocity = pendulum.getVelocity()

    
    print("Position:", position)
    print("Velocity:", velocity)

    
    world.update(time_step)


world.destroy()
engine.shutdown()