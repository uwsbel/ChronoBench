import pychrono as pc
import pychrono.trajectory as pt
import pychrono.visualization.irrlicht as vis
import numpy as np
import time


pc.init()
vis.init()


sim = pc.Simulation()
sim.setVisualStyle("Pendulum")


ground = pc.Body("ground", pc.Shape.createStaticSphere(1, 1, 1))
sim.addBody(ground)


mass = 1  
length = 5  
moment_inertia = mass * (length**2)  
pendulum = pc.Body("pendulum", pc.Shape.createSphere(0.5, 0.5, 0.5))
pendulum.setMass(mass)
pendulum.setMomentOfInertia(moment_inertia, pc.g, pc.g)

pendulum.setPosition([0, 0, 5])


joint = pc.RevoluteJoint("pendulum_joint", pendulum, ground, pc.Vec3(0, 0, 0), pc.Vec3(0, 0, 1))
sim.addJoint(joint)


renderer = vis.IrrlichtRenderer()
renderer.setCameraDistance(10)
renderer.setCameraLookAt([0, 0, 10])
renderer.setLightDirection([0, 1, 0])
renderer.setColorBackground([0.1, 0.1, 0.1])


renderer.add(pendulum, "Pendulum", 1, 1, 1, True)



spring = pc.HarmonicPotentialSpring("spring", pendulum, 0.5, 0.5, pc.Vec3(0, 0, -gravity), 0.1)
sim.addForce(spring)


logger = pc.Logger("PendulumData", pc.Logger.BOTH_FILE_AND_SCREEN, 100, 0.1)
logger.add(pendulum, "position", pc.Logger.POSITION, 0.1)
logger.add(pendulum, "velocity", pc.Logger.VELOCITY, 0.1)


sim.setGravity(pc.Vec3(0, -9.81, 0))
sim.setTimeStep(0.001)
sim.setRunTime(1000)  


sim.run()


vis.close()
pc.shutdown()