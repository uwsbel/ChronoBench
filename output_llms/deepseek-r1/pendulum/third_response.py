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

# Add a visualization cylinder to the ground
ground_cyl = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.AddVisualShape(ground_cyl, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1)))

# Create first pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum
pend_1.SetMass(1)  # Set the mass of the pendulum (in kg)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the first pendulum
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)  # Cylinder with radius 0.2 and height 2
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.ChVector3d(1, 0, 0), 
                                            chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the pendulum (center of mass) in the absolute frame
# The pendulum is initially horizontal, pointing towards the positive X direction
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# Create a revolute joint to connect the pendulum to the ground
# The revolute joint allows rotation around a single axis (the Z-axis in this case)
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, 
                chrono.ChFrameD(chrono.ChVector3d(0, 0, 1), 
                              chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)

# Create second pendulum body and add it to the simulation
pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))

# Add visualization to the second pendulum
cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_2.SetColor(chrono.ChColor(0, 0, 0.6))  # Different color (blue)
pend_2.AddVisualShape(cyl_2, chrono.ChFrameD(chrono.ChVector3d(1, 0, 0), 
                                           chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set initial position of second pendulum (center of mass)
pend_2.SetPos(chrono.ChVector3d(3, 0, 1))  # Attached to end of first pendulum

# Create revolute joint between first and second pendulum
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, 
                chrono.ChFrameD(chrono.ChVector3d(2, 0, 1), 
                              chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Double Pendulum Simulation')  # Updated title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  # Add a camera to the visualization
vis.AddTypicalLights()  # Add typical lights for better visualization

# Simulation loop
log_info = True  # Flag to control logging
while vis.Run():
    vis.BeginScene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End the scene
    sys.DoStepDynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds

    # Log information after 1 second of simulation time
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()  # Get position of first pendulum
        pos_2 = pend_2.GetPos()  # Get position of second pendulum
        
        print("t = ", sys.GetChTime())
        print("Pendulum 1 position: ", pos_1.x, "  ", pos_1.y)
        print("Pendulum 2 position: ", pos_2.x, "  ", pos_2.y)
        
        lin_vel_1 = pend_1.GetPosDt()  # Linear velocity of first pendulum
        lin_vel_2 = pend_2.GetPosDt()  # Linear velocity of second pendulum
        print("Pendulum 1 velocity: ", lin_vel_1.x, "  ", lin_vel_1.y)
        print("Pendulum 2 velocity: ", lin_vel_2.x, "  ", lin_vel_2.y)
        
        log_info = False  # Disable further logging