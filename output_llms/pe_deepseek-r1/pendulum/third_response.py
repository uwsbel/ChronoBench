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
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

# Create first pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum
pend_1.SetMass(1)  # Set the mass of the pendulum (in kg)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add visualization cylinder to the first pendulum
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)  # Cylinder with radius 0.2 and height 2
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set initial position of first pendulum (center of mass)
pend_1.SetPos(chrono.ChVector3d(1, 0, 0))  # Changed Z to 0 for 2D motion

# Create revolute joint to connect first pendulum to ground
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)

# Create second pendulum body and add it to the simulation
pend_2 = chrono.ChBody()
sys.Add(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))

# Add visualization cylinder to the second pendulum
cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_2.SetColor(chrono.ChColor(0, 0.6, 0))  # Green color for distinction
pend_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set initial position of second pendulum (center of mass)
pend_2.SetPos(chrono.ChVector3d(3, 0, 0))  # Positioned at end of first pendulum

# Create revolute joint to connect pendulums
rev_2 = chrono.ChLinkLockRevolute()
# Joint located at end of first pendulum (2m from its center)
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(chrono.ChVector3d(2, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Simulation')  # Updated title
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Simulation loop
log_info = True
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)  # Time step of 0.001 seconds

    # Log information after 1 second of simulation time
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()
        pos_2 = pend_2.GetPos()
        print("t = ", sys.GetChTime())
        print("Pendulum 1 position: ", pos_1.x, ", ", pos_1.y)
        print("Pendulum 2 position: ", pos_2.x, ", ", pos_2.y)
        
        lin_vel_1 = pend_1.GetPosDt()
        lin_vel_2 = pend_2.GetPosDt()
        print("Pendulum 1 velocity: ", lin_vel_1.x, ", ", lin_vel_1.y)
        print("Pendulum 2 velocity: ", lin_vel_2.x, ", ", lin_vel_2.y)
        
        log_info = False