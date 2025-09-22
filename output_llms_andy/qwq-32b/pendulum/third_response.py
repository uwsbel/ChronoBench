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
ground.AddVisualShape(cyl_ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.QIDENTITY))

# Create first pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum
pend_1.SetMass(1)  # Set the mass of the pendulum (in kg)
# Correct inertia tensor calculation for a cylinder (radius 0.2, length 2)
pend_1.SetInertiaXX(chrono.ChVectorD(
    (1/12)*1*(2)**2,  # Ixx (moment around cylinder axis)
    0.5*1*(0.2)**2,   # Iyy (perpendicular to cylinder axis)
    0.5*1*(0.2)**2    # Izz (perpendicular to cylinder axis)
))

# Add a visualization cylinder to the first pendulum
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)  # Cylinder with radius 0.2 and height 2
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 0),
    chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))  # Rotate around Y-axis by 90 degrees
))

# Set the initial position of the first pendulum (center of mass)
pend_1.SetPos(chrono.ChVectorD(1, 0, 1))

# Create a revolute joint to connect the first pendulum to the ground
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(
    ground,
    pend_1,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.QIDENTITY),  # Position in ground frame
    chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0), chrono.QIDENTITY)  # Position relative to pend_1 (center at (1,0,1))
)
sys.AddLink(rev_1)

# Create the second pendulum body and add it to the simulation
pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
# Correct inertia tensor calculation for the second pendulum (same as first)
pend_2.SetInertiaXX(chrono.ChVectorD(
    (1/12)*1*(2)**2,
    0.5*1*(0.2)**2,
    0.5*1*(0.2)**2
))

# Set the initial position of the second pendulum (center of mass)
pend_2.SetPos(chrono.ChVectorD(3, 0, 1))  # Offset by 2m along X from first pendulum's end

# Add a visualization cylinder to the second pendulum
cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_2.SetColor(chrono.ChColor(0, 0.6, 0))
pend_2.AddVisualShape(cyl_2, chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 0),
    chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))
))

# Create revolute joint between first and second pendulums
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(
    pend_1,
    pend_2,
    chrono.ChFrameD(chrono.ChVectorD(1, 0, 0), chrono.QIDENTITY),  # Position relative to pend_1 (end point)
    chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0), chrono.QIDENTITY)  # Position relative to pend_2 (end point)
)
sys.AddLink(rev_2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()

# Simulation loop
log_info = True
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)  # Time step of 0.001 seconds

    if log_info and sys.GetChTime() > 1:
        print(f"t = {sys.GetChTime():.2f}")
        print(f"Pendulum 1 Position: ({pend_1.GetPos().x:.2f}, {pend_1.GetPos().y:.2f})")
        print(f"Pendulum 1 Velocity: ({pend_1.GetPos_dt().x:.2f}, {pend_1.GetPos_dt().y:.2f})")
        print(f"Pendulum 2 Position: ({pend_2.GetPos().x:.2f}, {pend_2.GetPos().y:.2f})")
        print(f"Pendulum 2 Velocity: ({pend_2.GetPos_dt().x:.2f}, {pend_2.GetPos_dt().y:.2f})")
        log_info = False