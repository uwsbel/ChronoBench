import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # g = 9.81 m/s^2

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add visualization for the ground
ground.AddVisualShape(chrono.ChVisualShapeCylinder(0.2, 0.4), 
                       chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

# Create the first pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum
pend_1.SetMass(1)  # Set the mass of the pendulum (in kg)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add visualization for the first pendulum
pend_1.AddVisualShape(chrono.ChVisualShapeCylinder(0.2, 2), 
                       chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
pend_1.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0, 0))  # Set color for first pendulum

# Set the initial position of the first pendulum (center of mass) in the absolute frame
# The pendulum is initially horizontal, pointing towards the positive X direction
pend_1.SetPos(chrono.ChVector3d(0, 0, 1))

# Create a revolute joint to connect the first pendulum to the ground
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)

# Create the second pendulum body and add it to the simulation
pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)  # The second pendulum can move
pend_2.EnableCollision(False)  # Disable collision detection for the second pendulum
pend_2.SetMass(1)  # Set the mass of the second pendulum (in kg)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add visualization for the second pendulum
pend_2.AddVisualShape(chrono.ChVisualShapeCylinder(0.2, 2), 
                       chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
pend_2.GetVisualShape(0).SetColor(chrono.ChColor(0.0, 0.5, 1.0))  # Set color for second pendulum (blue)

# Create a revolute joint to connect the second pendulum to the first one
rev_2 = chrono.ChLinkLockRevolute()
# The second pendulum is attached at the end of the first pendulum
pend_1_pos = pend_1.GetPos()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(pend_1_pos + chrono.ChVector3d(0, 0, 0.5), 
                                                  chrono.ChQuaterniond(0, 0, 1, 0)))
sys.AddLink(rev_2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Double Pendulum demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  # Add a camera to the visualization
vis.AddTypicalLights()  # Add typical lights for better visualization

# Add a visualization cylinder for the ground
ground.AddVisualShape(chrono.ChVisualShapeCylinder(0.2, 0.4), 
                       chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

# Simulation loop
log_info = True  # Flag to control logging
while vis.Run():
    vis.BeginScene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End the scene
    sys.DoStepDynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds

    # Log information after 1 second of simulation time
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()  # Get the position of the first pendulum
        print("t = ", sys.GetChTime())
        print("     ", pos_1.x, "  ", pos_1.y)
        print("     ", pos_1.z, "  ")  # Added z-coordinate for clarity
        lin_vel_1 = pend_1.GetPosDt()  # Get the linear velocity of the first pendulum
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        print("     ", lin_vel_1.z, "  ")  # Added z-coordinate for clarity
        pos_2 = pend_2.GetPos()  # Get the position of the second pendulum
        print("     ", pos_2.x, "  ", pos_2.y)
        print("     ", pos_2.z, "  ")  # Added z-coordinate for clarity
        log_info = False  # Disable further logging