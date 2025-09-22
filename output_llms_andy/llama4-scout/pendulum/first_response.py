import chrono as ch
import chrono.irrlicht as chirr

# Initialize PyChrono environment
ch.ChEngine.Initialize()

# Create the Irrlicht application
app = chirr.ChIrrApp(ch.ChSystem(), "Simple Pendulum", chirr.IrrlichtInfo())

# Create the ground body
ground = ch.ChBodyStatic()
ground.SetPos(ch.ChVector3d(0, 0, -10))

# Create the pendulum body
pendulum = ch.ChBody()
pendulum.SetMass(1.0)
pendulum.SetPos(ch.ChVector3d(0, 5, 0))
pendulum.SetRot(ch.ChQuaterniond(1, 0, 0, 0))

# Inertia of the pendulum (assuming a rod of length 10 and radius 0.1)
inertia = ch.ChMatrix33d(1.0)
inertia(0, 0) = 1.0  # xx
inertia(1, 1) = 1.0  # yy
inertia(2, 2) = 0.01  # zz
pendulum.SetInertia(inertia)

# Create a revolute joint
joint = ch.ChRevolute()
joint.Init(ground, pendulum, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))

# Add bodies and joint to the system
system = app.GetSystem()
system.Add(ground)
system.Add(pendulum)
system.Add(joint)

# Visualization
visualization = ch.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle("Simple Pendulum")
visualization.Initialize()

# Logging
def LogPendulumState(t):
    print(f"Time: {t}, Pendulum Position: {pendulum.GetPos()}, Pendulum Velocity: {pendulum.GetPos_dt()}")

# Simulation loop
while visualization.Run():
    system.DoStepDynamics(0.01)
    LogPendulumState(system.GetChTime())

    # Optional: Add visualization shapes for ground and pendulum
    visualization.BeginScene()
    visualization.EndScene()