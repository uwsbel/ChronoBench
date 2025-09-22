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
ground.AddVisualShape(ground_cyl, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))

# Create first pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection
pend_1.SetMass(1)  # Set mass (kg)

# Calculate proper inertia for cylinder (radius=0.2, length=2)
r = 0.2
L = 2
m = 1
Ixx = 0.5 * m * r**2
Iyy = (1/12) * m * (3*r**2 + L**2)
pend_1.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Iyy))

# Add visualization to first pendulum
pend1_cyl = chrono.ChVisualShapeCylinder(0.2, 2)
pend1_cyl.SetColor(chrono.ChColor(0.6, 0, 0))  # Red color
pend_1.AddVisualShape(pend1_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Create revolute joint connecting pendulum to ground
joint_pos = chrono.ChVector3d(0, 0, 1)  # Joint position
pend_1.SetPos(joint_pos + chrono.ChVector3d(1, 0, 0))  # Center position

rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(joint_pos, chrono.QUNIT))
sys.AddLink(rev_1)

# Create second pendulum body
pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Iyy))  # Same inertia as first pendulum

# Add visualization to second pendulum
pend2_cyl = chrono.ChVisualShapeCylinder(0.2, 2)
pend2_cyl.SetColor(chrono.ChColor(0, 0.6, 0))  # Green color
pend_2.AddVisualShape(pend2_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Position second pendulum at end of first pendulum
pend_2.SetPos(joint_pos + chrono.ChVector3d(2, 0, 0))  # Center position

# Create revolute joint between pendulums
joint2_pos = joint_pos + chrono.ChVector3d(2, 0, 0)  # End of first pendulum
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(joint2_pos, chrono.QUNIT))
sys.AddLink(rev_2)

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 8))  # Adjusted camera position
vis.AddTypicalLights()

# Simulation loop
log_info = True
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)  # Time step 0.001 seconds

    # Log information after 1 second
    if log_info and sys.GetChTime() > 1:
        print(f"t = {sys.GetChTime():.3f} s")
        print(f"Pendulum 1 position: {pend_1.GetPos().x:.3f}, {pend_1.GetPos().y:.3f}")
        print(f"Pendulum 2 position: {pend_2.GetPos().x:.3f}, {pend_2.GetPos().y:.3f}")
        log_info = False