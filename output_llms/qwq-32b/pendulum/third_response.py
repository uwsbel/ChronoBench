import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2)
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))  # g = 9.81 m/s^2

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization cylinder to the ground
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.AddVisualShape(cyl_ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)))

# Create first pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum
pend_1.SetMass(1)  # Set the mass of the pendulum (in kg)
pend_1.SetInertiaXX(chrono.ChVectorD(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the first pendulum
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)  # Cylinder with radius 0.2 and height 2
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngY(chrono.CH_PI_2)))

# Set the initial position of the first pendulum (center of mass) in the absolute frame
pend_1.SetPos(chrono.ChVectorD(1, 0, 1))

# Create revolute joint between ground and first pendulum
rev_1 = chrono.ChLinkLockRevolute()
frame_ground = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0))
frame_pend1 = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
rev_1.Initialize(ground, pend_1, frame_ground, frame_pend1)
sys.AddLink(rev_1)

# Create second pendulum body and add it to the simulation
pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetMass(1)  # Set the mass of the second pendulum (in kg)
pend_2.SetInertiaXX(chrono.ChVectorD(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)

# Add a visualization cylinder to the second pendulum
cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)  # Cylinder with radius 0.2 and height 2
cyl_2.SetColor(chrono.ChColor(0, 0.6, 0))  # Different color for the second pendulum
pend_2.AddVisualShape(cyl_2, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngY(chrono.CH_PI_2)))

# Set the initial position of the second pendulum (center of mass) in the absolute frame
pend_2.SetPos(chrono.ChVectorD(3, 0, 1))  # Positioned at the end of the first pendulum's cylinder

# Create revolute joint between first and second pendulum
rev_2 = chrono.ChLinkLockRevolute()
frame_pend1_joint = chrono.ChFrameD(chrono.ChVectorD(1, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))  # Position on first pendulum's local frame
frame_pend2_joint = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))  # Position on second pendulum's local frame
rev_2.Initialize(pend_1, pend_2, frame_pend1_joint, frame_pend2_joint)
sys.AddLink(rev_2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Double Pendulum Simulation')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVectorD(0, 3, 6))  # Add a camera to the visualization
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
        pos_1 = pend_1.GetPos()  # Get the position of the first pendulum
        pos_2 = pend_2.GetPos()  # Get the position of the second pendulum
        print("t = ", sys.GetChTime())
        print("Pendulum 1: ", pos_1.x, pos_1.y)
        print("Pendulum 2: ", pos_2.x, pos_2.y)
        lin_vel_1 = pend_1.GetPos_dt()  # Get the linear velocity of the first pendulum
        lin_vel_2 = pend_2.GetPos_dt()  # Get the linear velocity of the second pendulum
        print("Vel 1: ", lin_vel_1.x, lin_vel_1.y)
        print("Vel 2: ", lin_vel_2.x, lin_vel_2.y)
        log_info = False  # Disable further logging