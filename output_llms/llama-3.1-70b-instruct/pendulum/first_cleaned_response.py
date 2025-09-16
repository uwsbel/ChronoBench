import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


mysystem = chrono.ChSystemNSC()
mysystem.SetStep(0.005)


ground = chrono.ChBodyEasyBox(mysystem, 4, 1, 4, 1000)
ground.SetPos(chrono.ChVectorD(0, -2, 0))
ground.SetBodyFixed(True)


pendulum = chrono.ChBodyEasyCylinder(mysystem, 0.5, 2, 1000)
pendulum.SetMass(10)
pendulum.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
pendulum.SetPos(chrono.ChVectorD(0, 2, 0))


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
mysystem.Add(revolute_joint)


vis = chronoirr.ChVisualSystemIrrlicht(mysystem)


vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Pendulum Simulation")
vis.SetCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


while vis.Run():
    mysystem.DoStepDynamics(0.005)
    
    
    print(f"Pendulum position: {pendulum.GetPos().x} {pendulum.GetPos().y} {pendulum.GetPos().z}")
    print(f"Pendulum velocity: {pendulum.GetV().x} {pendulum.GetV().y} {pendulum.GetV().z}")
    print()


mysystem.SetStep(0)