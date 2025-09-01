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
# It's better to keep collision detection enabled but set the geometry to something simple
ground.SetCollide(True) 
ground.SetBodyFixed(True)

# Add a visualization cylinder to the ground
cyl_g = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.AddVisualShape(cyl_g, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1))) 
# Use ChFrameD instead of ChFramed 

# --- Pendulum 1 ---
# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetBodyFixed(False)  # The pendulum can move
# It's better to keep collision detection enabled but set the geometry to something simple
pend_1.SetCollide(True) 
pend_1.SetMass(1)  # Set the mass of the pendulum (in kg)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the pendulum
cyl_1 = chrono.ChVisualShapeCylinder(0.1, 2)  # Cylinder with radius 0.2 and height 2
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngY(chrono.CH_PI/2)))
# Use Q_from_AngY instead of QuatFromAngleY, ChFrameD instead of ChFramed

# Set the initial position of the pendulum (center of mass) in the absolute frame
# The pendulum is initially horizontal, pointing towards the positive X direction
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# Create a revolute joint to connect the pendulum to the ground
# The revolute joint allows rotation around a single axis (the Z-axis in this case)
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
sys.AddLink(rev_1)

# --- Pendulum 2 ---
pend_2 = chrono.ChBody()
sys.Add(pend_2)
pend_2.SetBodyFixed(False)
pend_2.SetCollide(True)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))

# Add visualization for pendulum 2
cyl_2 = chrono.ChVisualShapeCylinder(0.1, 2)
cyl_2.SetColor(chrono.ChColor(0, 0.6, 0))
pend_2.AddVisualShape(cyl_2, chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngY(chrono.CH_PI/2)))

# Set the initial position of the second pendulum
pend_2.SetPos(chrono.ChVector3d(2, 0, 1))  # Initially hanging vertically below pendulum 1

# Connect the second pendulum to the first one
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChCoordsysD(chrono.ChVector3d(2, 0, 1), chrono.QUNIT))
sys.AddLink(rev_2)


# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Double Pendulum')  # Set the window title
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
        pos_1 = pend_1.GetPos()  # Get the position of the pendulum
        pos_2 = pend_2.GetPos()
        print("t = ", sys.GetChTime())
        print("Pendulum 1: ", pos_1.x, "  ", pos_1.y)
        print("Pendulum 2: ", pos_2.x, "  ", pos_2.y)
        log_info = False  # Disable further logging