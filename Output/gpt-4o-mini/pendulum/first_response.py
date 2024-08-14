import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# Initialize the PyChrono environment
chrono.ChSystemNSC.SetNumThreads(1)
system = chrono.ChSystemNSC()

# Create a visualization system using Irrlicht
application = chronoirr.ChIrrApp(system, "Simple Pendulum Simulation", chronoirr.dimension2du(800, 600))
application.AddLogo()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 2, -5))
application.AddLight(chrono.ChVectorD(0, 10, -10), chrono.ChColor(1, 1, 1))
application.SetShowInfos(True)

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Create the pendulum body
pendulum_length = 2.0
pendulum_mass = 1.0
pendulum_inertia = (1/3) * pendulum_mass * pendulum_length**2

pendulum = chrono.ChBodyEasyBox(0.1, pendulum_length, 0.1, pendulum_mass, True, True)
pendulum.SetPos(chrono.ChVectorD(0, -pendulum_length / 2, 0))
pendulum.SetInertiaXX(chrono.ChVectorD(pendulum_inertia, pendulum_inertia, pendulum_inertia))
system.Add(pendulum)

# Create a revolute joint between the ground and the pendulum
joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(joint)

# Set initial conditions
pendulum.SetPos(chrono.ChVectorD(0, -pendulum_length / 2, 0))
pendulum.SetPos_dt(chrono.ChVectorD(0, 0, 0))  # Initial velocity

# Simulation parameters
time_step = 0.01
end_time = 10.0

# Main simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    # Step the simulation
    system.DoStepDynamics(time_step)

    # Log the pendulum's position and velocity
    pendulum_position = pendulum.GetPos()
    pendulum_velocity = pendulum.GetPos_dt()
    print(f"Time: {system.GetChTime():.2f}, Position: {pendulum_position}, Velocity: {pendulum_velocity}")

    # Break the loop if the end time is reached
    if system.GetChTime() >= end_time:
        break

# Cleanup
application.Close()