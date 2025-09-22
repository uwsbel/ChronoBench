import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set gravitational acceleration to simulate moon gravity (1.62 m/s^2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization cylinder to the ground
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum
pend_1.SetMass(2)  # Set the mass to 2 kg
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Set the inertia tensor

# Add a visualization cylinder to the pendulum
cyl_pend = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Radius 0.1, height 1.5
cyl_pend.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color
pend_1.AddVisualShape(cyl_pend, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Add a visualization sphere for the joint
sphere_joint = chrono.ChVisualShapeSphere(0.2)  # Radius 0.2
sphere_joint.SetColor(chrono.ChColor(0, 0.6, 0))  # Green color
pend_1.AddVisualShape(sphere_joint, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

# Set the initial position of the pendulum (center of mass)
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# Set initial angular velocity for the pendulum
pend_1.SetWvelAngularLocal(chrono.ChVector3d(0, 0, 5))  # Angular velocity around Z-axis

# Create a spherical joint to connect the pendulum to the ground
spher_1 = chrono.ChLinkLockSpherical()
spher_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(spher_1)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size
vis.SetWindowTitle('Modified Pendulum Simulation')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add logo
vis.AddSkyBox()  # Add skybox
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  # Add camera
vis.AddTypicalLights()  # Add lights

# Simulation loop
log_info = True  # Flag to control logging
while vis.Run():
    vis.BeginScene()  # Begin scene rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End scene
    sys.DoStepDynamics(1e-3)  # Advance simulation by 0.001 seconds

    # Log information after 1 second of simulation time
    if log_info and sys.GetChTime() > 1:
        pos = pend_1.GetPos()  # Get position
        print("t = ", sys.GetChTime())
        print("Position: ", pos.x, pos.y)
        lin_vel = pend_1.GetPosDt()  # Get linear velocity
        print("Velocity: ", lin_vel.x, lin_vel.y)
        log_info = False  # Disable further logging