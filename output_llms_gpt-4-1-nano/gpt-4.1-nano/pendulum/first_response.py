import pychrono.core as chrono
import pychrono.irrlicht as irr

# 1. Initialize PyChrono system and visualization
# Create the physical system
system = chrono.ChSystemNSC()

# Create the Irrlicht application for visualization
application = irr.ChIrrApp(system, "Simple Pendulum Simulation", irr.dimension2du(800, 600))
application.GetSceneManager().AddSkyBox()
application.GetCamera().SetChPos(chrono.ChVectorD(0, 0, 1))
application.AssetBindAll()
application.AssetUpdateAll()

# 2. Create ground body
ground = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True)
ground.SetPosition(chrono.ChVectorD(0, -0.05, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# 3. Create pendulum body
mass = 1.0
radius = 0.05
length = 1.0

# Inertia for a solid sphere approximated as a cylinder
pendulum = chrono.ChBodyEasyCylinder(radius, length, mass, True, True)
pendulum.SetPos(chrono.ChVectorD(0, -length / 2, 0))
system.Add(pendulum)

# 4. Connect pendulum to ground with a revolute joint
pivot_point = chrono.ChVectorD(0, 0, 0)
revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(pivot_point))
system.Add(revolute_joint)

# 5. Set initial conditions
# For a simple pendulum, set initial angle (e.g., 30 degrees)
initial_angle_deg = 30
initial_angle_rad = chrono CH_C_PI/6  # 30 degrees in radians
pendulum.SetPos(chrono.ChVectorD(length * chrono.ChMathsProjects.Cos(initial_angle_rad),
                                 -length * chrono.ChMathsProjects.Sin(initial_angle_rad),
                                 0))
# Initial angular velocity
pendulum.SetW(chrono.ChVectorD(0, 0, 0))

# 6. Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# 7. Simulation parameters
time_step = 0.005
total_time = 10  # seconds

# 8. Run the simulation with visualization and logging
application.SetTimestep(time_step)
application.StartScene()
application.DrawAll()

current_time = 0.0
while current_time < total_time:
    system.DoStepDynamics(time_step)
    application.BeginScene()
    application.DrawAll()

    # Log the position and velocity of the pendulum bob
    pos = pendulum.GetPos()
    vel = pendulum.GetW()
    print(f"Time: {current_time:.3f} s - Pendulum position: {pos}, angular velocity: {vel}")

    application.EndScene()

    current_time += time_step
    application.GetVideoDriver().DrawLine(chrono.ChVectorD(0,0,0), pos, irr.SColor(255,255,0,0))
    
application.GetDevice().closeDevice()