import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the PyChrono system
chrono.SetChronoDataPath('path/to/chrono/data/')
my_system = chrono.ChSystemNSC()

# Create a ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
my_system.Add(ground)

# Create a pendulum body
pendulum_radius = 0.1  # Radius of the pendulum bob
pendulum_length = 1.0  # Length of the pendulum rod
pendulum_mass = 1.0    # Mass of the pendulum
pendulum_inertia = chrono.ChVectorD(0.5 * pendulum_mass * pendulum_radius**2, 0.5 * pendulum_mass * pendulum_radius**2, 0.5 * pendulum_mass * pendulum_radius**2)
pendulum = chrono.ChBodyEasySphere(pendulum_radius, pendulum_mass, True, True, pendulum_inertia)
pendulum.SetPos(chrono.ChVectorD(0, -pendulum_length, 0))
my_system.Add(pendulum)

# Create a revolute joint to connect the pendulum to the ground
revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
my_system.Add(revolute_joint)

# Initialize the Irrlicht visualization system
application = chronoirr.ChIrrApp(my_system, 'Pendulum Simulation', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
application.AddTypicalCamera(chronoirr.vector3df(0, -2, 0), chronoirr.vector3df(0, 0, 0))
application.AddLightWithShadow(chronoirr.vector3df(2, 2, 2), chronoirr.vector3df(0, 0, 0), 10, 5, 10)
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
time_step = 0.01  # Simulation time step
log_interval = 50  # Interval for logging (every 50 steps)
application.SetTimestep(time_step)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    # Log the pendulum's position and velocity periodically
    if my_system.GetChTime() % (log_interval * time_step) < time_step:
        pendulum_pos = pendulum.GetPos()
        pendulum_vel = pendulum.GetPos_dt()
        print(f"Time: {my_system.GetChTime():.2f}, Position: {pendulum_pos}, Velocity: {pendulum_vel}")