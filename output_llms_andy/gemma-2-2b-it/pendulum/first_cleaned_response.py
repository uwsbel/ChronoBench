import pychrono as chrono
import pychrono.irrlicht as irrlicht
import numpy as np


chrono.init()


engine = irrlicht.IrrlichtEngine()


world = chrono.World()


ground = chrono.Body(chrono.BodyType.Ground)
ground.setMass(100)
ground.setPosition(0, 0, 0)
world.addBody(ground)


pendulum = chrono.Body(chrono.BodyType.RigidBody)
pendulum.setMass(1)
pendulum.setInertia(chrono.Inertia(1, 0, 0))  
pendulum.setPosition(0, 1, 0)  
pendulum.setVelocity(0, 0, 0)  
pendulum.setAngularVelocity(0, 0, 0)  


joint = chrono.RevoluteJoint(pendulum, ground)
world.addBody(pendulum)
world.addBody(ground)
world.addJoint(joint)


engine.setCameraPosition(0, 0, 10)
engine.setCameraTarget(0, 0, 0)
engine.setCameraZoom(10)


dt = 0.01  
duration = 10  


while engine.isRunning():
    
    world.update(dt)

    
    print("Position:", pendulum.getPosition())
    print("Velocity:", pendulum.getVelocity())

    
    engine.render()


engine.shutdown()