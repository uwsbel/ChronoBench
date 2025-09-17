from pychrono import *
from pychrono.pchTime import *
from pychrono.pchPhysics import *
from pychrono.pchME import *
from pychrono.pchVisualization import *
from pychrono.pchVisualization.irrlicht import *


chTimeInit()
chPhysicsInit()


dt = 1e-5  
t_final = 0.1  
t = 0.0  


sim = Simulation()


beam_length = 1.0  
beam_radius = 0.01  
beam = FEAModel("Beam", beam_length, beam_radius)


E = 210e9  
nu = 0.3  
beam.material = Material(E, nu)


def motor_function(x, v, a):
    F = -1000 * x  
    return F

beam.motor = Motor(motor_function)


sim.add(beam)



beam1 = FEAModel("Beam1", 0.5, 0.01)
beam2 = FEAModel("Beam2", 0.5, 0.01)


hinge = HingeConstraint(beam1, beam2, CHVector(0, 0, 0))


sim.add(beam1)
sim.add(beam2)
sim.add(hinge)


beam.setPosition(0,0,0, 0, x, x,  
beam1.setPosition( 0.5, 0.0, 0.0)
beam2.setPosition( 0.5, 0.0, 0.0)


vis = IrrlichtVisual()


vis.addObject(sim)


sim.setSolver(ChSolverNewton(pCHExplicit))
sim.setTimestepper(ChTimestepperVerlet(pCHExplicit))


while sim.getTime() < t_final:
    sim.doStep(dt)
    vis.update()
    sim.render()


vis.end()
sim.finalize()