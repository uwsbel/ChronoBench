import pychrono

import pychrono.ChSystemNSC as ChSystemNSC




ch = ChSystemNSC()






ground = pychrono.ChBody()

ground.SetStaticMass(1.0)

ground.SetName("Ground")

ch.AddBody(ground)




ball = pychrono.ChBody()

ball.SetMass(1.0)

ball.SetName("Ball")

ch.AddBody(ball)




ball.SetPos(pychrono.ChVector(0, 0, 10)

ball.SetLinearVel(pychrono.ChVector(0, 0, 0))




ch.SetGravity(pychrono.ChVector(0, 0, -9.81))




ball.SetFriction(0.5)

ground.SetFriction(0.5)




contact = pychrono.ChContactGeometry()

contact.SetFriction(0.5)

contact.SetRestitution(0.8)




ch.AddContact(ball, ground, contact)




sim_time = 5.0

time_step = 0.01




ch.SetSolverIterations(10)

ch.SetSolverIterations(10)

ch.SetSolverTolerance(1e-6)

ch.SetSolverTolerance(1e-6)

ch.SetSolverMaxIterations(100)

ch.SetSolverMaxTimeStep(0.01)




ch.DoStepDynamics(time_step)




while ch.GetChTime() < sim_time:

    ch.DoStepDynamics(time_step)




print("Final position of the ball:", ball.GetPos())