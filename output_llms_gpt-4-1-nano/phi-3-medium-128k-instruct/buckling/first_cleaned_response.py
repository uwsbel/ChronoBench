import pychrono

import pychrono.fea as fea

import pychrono.irrlicht as irr

import pychrono.solver as solver

import pychrono.rigidbody as rb

import pychrono.sparse as sp

import pychrono.contact as contact




chrono = pychrono.ChLinkCreateStatic()




length = 2.0

width = 0.1

thickness = 0.1

E = 210e9  

I = (width * thickness**3) / 12  




beam = fea.ChBeam(chrono, length, width, thickness, E, I)




def motor_function(time):

    

    return 1000 * pychrono.sin(2 * pychrono.pi * time)




motor = chrono.ChLinkCreatePrismatic(chrono)

motor.SetMotorFunction(motor_function)

beam.AddLink(motor)




solver = solver.ChSolver64()

timestepper = solver.ChSolver64(chrono, 0.001, 0.0001)




irr.initialize(chrono)




irr.AddModel(beam)






fixed_end = chrono.ChBody(chrono)

fixed_end.SetMass(0)

fixed_end.SetPosition(0, 0, 0)

fixed_end.SetBody(beam)




chrono.SetSolver(solver)

chrono.SetTimestepper(timestepper)




while chrono.GetTimestep() < 10:

    chrono.DoStepDynamics(1)

    irr.render()




irr.end()