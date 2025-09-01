import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # g = 9.81 m/s²

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization cylinder to the ground
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1)))

# Create first pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection
pend_1.SetMass(1)  # Mass (kg)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Inertia tensor (kg·m²)

# Add visualization cylinder to first pendulum
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)  # Radius 0.2, height 2
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Red color
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0), 
                                          chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set initial position of first pendulum (center of mass)
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# Create revolute joint between ground and first pendulum
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, 
                chrono.ChFrameD(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)

# Create second pendulum body and add it to the simulation
pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))

# Add visualization cylinder to second pendulum
cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_2.SetColor(chrono.ChColor(0, 0.6, 0))  # Green color
pend_2.AddVisualShape(cyl_2, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0), 
                                          chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set initial position of second pendulum (center of mass)
pend_2.SetPos(chrono.ChVector3d(3, 0, 1))

# Create revolute joint between first and second pendulums
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, 
                chrono.ChFrameD(chrono.ChVector3d(2, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Demo')
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
    sys.DoStepDynamics(1e-3)

    # Log information after 1 second of simulation
    if log_info and sys.GetChTime() > 1:
        pos1 = pend_1.GetPos()
        pos2 = pend_2.GetPos()
        vel1 = pend_1.GetPosDt()
        vel2 = pend_2.GetPosDt()
        
        print(f"t = {sys.GetChTime():.3f}")
        print(f"Pendulum 1 - Position: ({pos1.x:.3f}, {pos1.y:.3f})")
        print(f"Pendulum 2 - Position: ({pos2.x:.3f}, {pos2.y:.3f})")
        print(f"Pendulum 1 - Velocity: ({vel1.x:.3f}, {vel1.y:.3f})")
        print(f"Pendulum 2 - Velocity: ({vel2.x:.3f}, {vel2.y:.3f})")
        log_info = False