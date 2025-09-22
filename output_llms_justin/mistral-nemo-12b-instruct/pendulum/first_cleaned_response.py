import pychrono as chrono
from pychrono.irrlicht import *


sim = chrono.ChSimulation()
irr = IrrlichtDirector(sim)


sys = chrono.ChSystemSMC()
sim.AddSystem(sys)


ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(ground)


pendulum = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1, True)
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))
sys.Add(pendulum)


joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 1))
sys.Add(joint)


irr.SetCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
irr.Add(ground)
irr.Add(pendulum)
irr.Add(joint)


def log_pendulum_state():
    pos = pendulum.GetPos()
    vel = pendulum.GetPos_dt()
    print(f"Pendulum position: {pos}, velocity: {vel}")


for i in range(100):
    sys.DoStepDynamics(0.01)
    if i % 10 == 0:
        log_pendulum_state()
    irr.Render()
    chrono.ChThreadSleep(0.01)

irr.Shutdown()