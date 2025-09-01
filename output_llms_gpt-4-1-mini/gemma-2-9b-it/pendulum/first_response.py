import pychrono as chrono
import pychrono.irrlicht as vis

# Initialize Chrono environment
chrono.SetChronoDataPath("path/to/chrono/data")  # Replace with your Chrono data path
system = chrono.ChSystemNSC()

# Create ground body
ground = chrono.ChBodyEasyBox(10, 10, 1, 1000)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)

# Create pendulum body
pendulum_mass = 1.0
pendulum_length = 1.0
pendulum_inertia = chrono.ChVectorD(0.1, 0.1, 0.1)  # Approximate inertia
pendulum = chrono.ChBodyEasySphere(pendulum_mass, pendulum_inertia)
pendulum.SetPos(chrono.ChVectorD(0, pendulum_length, 0))
system.Add(pendulum)

# Create revolute joint
joint = chrono.ChLinkLockRevolute(ground, pendulum,
                                  chrono.ChVectorD(0, 0, 0),
                                  chrono.ChVectorD(0, 0, 1))
system.Add(joint)

# Visualization setup
vis.Initialize(chrono.ChVectorD(800, 600, 0))
vis.Add(ground)
vis.Add(pendulum)
vis.SetCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))

# Simulation loop
dt = 0.01
while True:
    system.DoStepDynamics(dt)

    # Log pendulum position and velocity
    pendulum_pos = pendulum.GetPos()
    pendulum_vel = pendulum.GetRotSpeed()
    print(f"Position: {pendulum_pos}, Velocity: {pendulum_vel}")

    vis.Render()