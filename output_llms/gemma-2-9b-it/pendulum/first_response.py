from pychrono import ChSystem, ChBody, ChLink, ChMaterialSurface, ChVectorD, ChQuaternionD, ChFrame
from pychrono.irrlicht import ChIrrApp

# Initialize PyChrono system
system = ChSystem()

# Create ground body
ground = ChBody(mass=1e6)
ground.SetPos(ChVectorD(0, 0, 0))
system.AddBody(ground)

# Create pendulum body
pendulum_mass = 1.0
pendulum_inertia = ChVectorD(0.1, 0.1, 0.1)  # Approximate inertia tensor
pendulum = ChBody(mass=pendulum_mass)
pendulum.SetPos(ChVectorD(0, 1, 0))
pendulum.SetInertia(pendulum_inertia)
system.AddBody(pendulum)

# Create revolute joint
joint = ChLink(ground, pendulum, ChFrame(ChVectorD(0, 0, 0), ChQuaternionD(1, 0, 0, 0)))
joint.SetPos(ChVectorD(0, 0, 0))
joint.SetAxis(ChVectorD(0, 0, 1))  # Rotation axis
system.AddLink(joint)

# Set up visualization
app = ChIrrApp(system, 'Pendulum Simulation', windowSize=ChVectorD(800, 600))
app.AddCamera(ChVectorD(5, 5, 5))
app.SetChaseTarget(pendulum)
app.AssetBindAll()

# Simulation loop
dt = 0.01
while app.GetTimer() < 10:
    system.DoStepDynamics(dt)
    
    # Log pendulum position and velocity
    print(f"Time: {app.GetTimer()}, Pendulum Angle: {pendulum.GetPos().y}")
    
    app.DrawAll()
    app.GetDevice().sleep()