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

# Add a visualization cylinder to the ground for reference (optional)
# Note: ChVisualShapeCylinder is deprecated in newer versions; use AddVisualShape instead if needed
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

# Create the first pendulum body
pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum
pend_1.SetMass(1)  # Set the mass of the pendulum (in kg)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the first pendulum
cyl_1_vis = chrono.ChVisualShapeCylinder(0.2, 2)  # radius=0.2, height=2
cyl_1_vis.SetColor(chrono.ChColor(0.6, 0, 0))  # Set color to dark red
pend_1.AddVisualShape(cyl_1_vis, chrono.ChFramed(chrono.ChVector3d(0, -1, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the first pendulum's center of mass
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# Create the second (child) pendulum body
pend_2 = chrono.ChBody()
sys.Add(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))

# Add a visualization cylinder to the second pendulum
cyl_2_vis = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_2_vis.SetColor(chrono.ChColor(0, 0, 0.6))  # Set color to dark blue
pend_2.AddVisualShape(cyl_2_vis, chrono.ChFramed(chrono.ChVector3d(0, -1, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the second pendulum (attached to the first)
# Position it at the end of the first pendulum
pend_2.SetPos(chrono.ChVector3d(1, -2, 1))  # Assuming pend_1 length = 2 units downward

# Create a revolute joint to connect the first pendulum to the ground
rev_ground_to_pend1 = chrono.ChLinkLockRevolute()
# Pioneer the joint at the top of pend_1
rev_ground_to_pend1.Initialize(
    ground,
    pend_1,
    chrono.ChFrameD(chrono.ChVector3d(1, 0, 1), chrono.Quat(1, 0, 0, 0))
)
sys.AddLink(rev_ground_to_pend1)

# Create a revolute joint to connect the second pendulum to the first
rev_pend1_to_pend2 = chrono.ChLinkLockRevolute()
# Connect at the end of pend_1
rev_pend1_to_pend2.Initialize(
    pend_1,
    pend_2,
    chrono.ChFrameD(chrono.ChVector3d(1, -2, 1), chrono.Quat(1, 0, 0, 0))
)
sys.AddLink(rev_pend1_to_pend2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Simulation')
vis.Initialize()
# Note: Make sure logo image exists or comment out the next line if not needed
# vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Simulation loop
log_info = True
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    # Log info after 1 second of simulation
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()
        print("t = ", sys.GetChTime())
        print("Pendulum 1 position: ", pos_1.x, " ", pos_1.y, " ", pos_1.z)
        lin_vel_1 = pend_1.GetPosDt()
        print("Pendulum 1 velocity: ", lin_vel_1.x, " ", lin_vel_1.y, " ", lin_vel_1.z)
        pos_2 = pend_2.GetPos()
        print("Pendulum 2 position: ", pos_2.x, " ", pos_2.y, " ", pos_2.z)
        lin_vel_2 = pend_2.GetPosDt()
        print("Pendulum 2 velocity: ", lin_vel_2.x, " ", lin_vel_2.y, " ", lin_vel_2.z)
        log_info = False