import pychoro as pc

import irrlicht as ir




pc.init()




scene = pc.SceneGraph()




ground = pc.Body(pc.WorldCoordinates, mass=0, friction=0.5, restitution=0.0)

scene.addBody(ground)




length = 1.0  

mass = 1.0    

inertia = pc.Inertia(mass * length**2 / 3)

pendulum = pc.Body(pc.WorldCoordinates, mass, inertia, restitution=0.0)

pendulum.setPosition(pc.Vector3(0, length, 0))

pendulum.setLinearVelocity(pc.Vector3(0, 0, 0))

pendulum.setAngularVelocity(pc.Vector3(0, 0, 0))

scene.addBody(pendulum)




joint = pc.Joint(pc.WorldCoordinates, pc.JointType.REVOLUTE, ground, pendulum)

scene.addJoint(joint)




vis = pc.VisualSystem(scene)

vis.setGlColor(1.0, 1.0, 1.0)




gravity = pc.Vector3(0, -9.81, 0)

scene.setGravity(gravity)




for i in range(1000):

    scene.update()

    vis.drawAll()

    if i % 100 == 0:

        print(f"Pendulum position: {pendulum.getPosition()}")

        print(f"Pendulum velocity: {pendulum.getLinearVelocity()}")




pc.quit()