import pychrono as chrono
from pychrono import irrlicht as chronoirr

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

# Create ground body
ground = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Pendulum parameters
mass = 1  # kg
radius = 0.1  # meters
length = 1  # meters
density = mass / (chrono.CH_PI * radius**2 * length)

# Create pendulum body
pendulum = chrono.ChBodyEasyCylinder(radius, length, density, True, True, chrono.ChAxisRotation.eX)
pendulum.SetPos(chrono.ChVectorD(-length / 2, 0, 0))
system.Add(pendulum)

# Create revolute joint at origin (0,0,0)
revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
system.AddLink(revolute_joint)

# Set up Irrlicht visualization
application = chronoirr.ChIrrApp(system, 'Pendulum Simulation', chronoirr.dimension2du(800, 600))
application.SetSkyBox()
application.SetCamera(chronoirr.vector3df(0, 1, 3))
application.SetShowInfos(True)
application.SetVSync(True)
application.SetAmbientLight(chronoirr.SColorf(0.4, 0.4, 0.4, 1))
application.AddTypicalLights()

# Simulation parameters
time_step = 0.001
log_interval = 0.05
next_log_time = 0.0

# Simulation loop
application.BeginScene()
application.DrawAll()
application.EndScene()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    
    # Log pendulum state
    current_time = system.GetChTime()
    if current_time >= next_log_time:
        pos = pendulum.GetPos()
        ang_vel = pendulum.GetWvel_abs()
        print(f"Time: {current_time:.2f}, Position: {pos}, Angular Velocity: {ang_vel}")
        next_log_time += log_interval
    
    application.EndScene()