import pychrono as chrono
import pychrono.irrlicht as irrlicht
import pychrono.physics as physics
import pychrono.math as math


chrono.init()


engine = irrlicht.IrrlichtEngine()


world = chrono.World()


ground = chrono.Body(chrono.BodyType.STATIC, chrono.Vector3(0, 0, 0))
ground.setMass(1000)
ground.setInertia(chrono.Vector3(0, 0, 0))
world.addBody(ground)


pendulum = chrono.Body(chrono.BodyType.DYNAMIC, chrono.Vector3(0, 5, 0))
pendulum.setMass(1)
pendulum.setInertia(chrono.Vector3(0, 0.1, 0))
pendulum.setFixedRotation(True)


joint = chrono.RevoluteJoint(pendulum, ground)
world.addBody(pendulum)
world.addBody(ground)
world.addJoint(joint)


pendulum.setPosition(chrono.Vector3(0, 5, 0))
pendulum.setVelocity(chrono.Vector3(0, 0, 0))


engine.addScene(world)
engine.run()


while True:
    
    position = pendulum.getPosition()
    velocity = pendulum.getVelocity()

    
    print(f"Position: {position}")
    print(f"Velocity: {velocity}")

    
    engine.update()