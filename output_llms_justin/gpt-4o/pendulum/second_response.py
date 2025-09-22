import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2)
sys.Set_G_acc(chrono.ChVector3d(0, -1.62, 0))  # g = 1.62 m/s^2 (moon gravity)

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetBodyFixed(True)  # The ground is fixed and does not move
ground.SetCollide(False)  # Disable collision detection for the ground

# Add a visualization cylinder to the ground
cyl_1 = chrono.ChCylinderShape()
cyl_1.GetCylinderGeometry().p1 = chrono.ChVector3d(0, 0, 0.2)
cyl_1.GetCylinderGeometry().p2 = chrono.ChVector3d(0, 0, -0.2)
cyl_1.GetCylinderGeometry().rad = 0.2
ground.AddVisualShape(cyl_1)

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetBodyFixed(False)  # The pendulum can move
pend_1.SetCollide(False)  # Disable collision detection for the pendulum
pend_1.SetMass(2)  # Set the mass of the pendulum (in kg)
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the pendulum
cyl_2 = chrono.ChCylinderShape()
cyl_2.GetCylinderGeometry().p1 = chrono.ChVector3d(0, -0.75, 0)
cyl_2.GetCylinderGeometry().p2 = chrono.ChVector3d(0, 0.75, 0)
cyl_2.GetCylinderGeometry().rad = 0.1
cyl_2.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.AddVisualShape(cyl_2)

# Set the initial position of the pendulum (center of mass) in the absolute frame
# The pendulum is initially horizontal, pointing towards the positive X direction
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# Set an initial angular velocity for the pendulum
pend_1.SetWvel_par(chrono.ChVector3d(0, 0, 1))

# Create a spherical joint to connect the pendulum to the ground
spherical_joint = chrono.ChLinkLockSpherical()
spherical_joint.Initialize(ground, pend_1, chrono.ChCoordsys(chrono.ChVector3d(0, 0, 1)))
sys.AddLink(spherical_joint)

# Add a visualization sphere to the joint
sphere = chrono.ChSphereShape()
sphere.GetSphereGeometry().rad = 0.2
ground.AddVisualShape(sphere, chrono.ChFrame(chrono.ChVector3d(0, 0, 1)))

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('ChBodyAuxRef demo')  # Set the window title
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
        print("t = ", sys.GetChTime())
        print("     ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPos_dt()  # Get the linear velocity of the pendulum
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        log_info = False  # Disable further logging