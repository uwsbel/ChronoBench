import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


mysystem = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(mysystem, 4, 2, 4, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, -2, 0))
ground.SetBodyFixed(True)


pendulum = chrono.ChBodyEasyCylinder(mysystem, 1, 2, 1000, True, False)
pendulum.SetPos(chrono.ChVectorD(0, 2, 0))
pendulum.SetMass(10)
pendulum.SetInertiaXX(chrono.ChVectorD(1, 1, 1))


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
mysystem.Add(revolute_joint)


force = chrono.ChForce()
force.SetForce(chrono.ChVectorD(0, 100, 0))
pendulum.AddForce(force)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Simple Pendulum Simulation")
vis.AttachSystem(mysystem)


stepper = chrono.ChTimestepperHHT(mysystem)
stepper.SetStep(0.01)

while vis.Run():
    mysystem.DoStepDynamics(stepper.GetStep())
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    print("Pendulum position: ", pendulum.GetPos())
    print("Pendulum velocity: ", pendulum.GetPos_dt())