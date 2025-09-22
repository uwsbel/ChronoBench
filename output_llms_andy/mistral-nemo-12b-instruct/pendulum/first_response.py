import pychrono as chrono
from pychrono.irrlicht import *

# Initialize PyChrono environment and core components
sim = chrono.ChSimulation()
irr = IrrlichtInterface()
sim.AddSystem(irr.GetSystem())

# Set up the physical systems and objects
ground = chrono.ChBodyEasyBox(10, 10, 1, 1000, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
sim.Add(ground)

pendulum = chrono.ChBody()
pendulum.SetMass(1)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.01))
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))
pendulum.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI / 2))
sim.Add(pendulum)

joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(0, 1, 0))
sim.Add(joint)

# Set necessary default parameters
gravity = chrono.ChVectorD(0, -9.81, 0)
sim.Set_Gravity(gravity)

# Visualization and logging
while sim.GetSystem().GetChTime() < 10:
    sim.DoStepDynamics(0.01)

    # Periodically log the pendulum's position and velocity
    if sim.GetSystem().GetChTime() % 0.1 < 0.01:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {sim.GetSystem().GetChTime():.2f}, Position: {pos}, Velocity: {vel}")

    # Render the simulation
    sim.GetSystem().Render()