import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemSMC()
visualization = chronoirr.ChVisualization(chrono.ChSystemSMC())



pendulum_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
pendulum_body.SetPos(chrono.ChVectorD(0, 0.5, 0))
pendulum_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
pendulum_body.SetBodyFixed(False)
chrono.ChSystemSMC().Add(pendulum_body)


fixed_body = chrono.ChBody()
fixed_body.SetBodyFixed(True)
chrono.ChSystemSMC().Add(fixed_body)

revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(pendulum_body, fixed_body, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
chrono.ChSystemSMC().Add(revolute_joint)



chrono.ChSystemSMC().Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


force_magnitude = 10
force_direction = chrono.ChVectorD(1, 0, 0)
pendulum_body.SetForce(force_magnitude * force_direction)


step_size = 0.001


simulation_time = 10
for i in range(int(simulation_time / step_size)):
    chrono.ChSystemSMC().DoStepDynamics(step_size)
    visualization.DrawAll()