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

# Add a visualization cylinder to the ground (fixed ChFrameD)
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_ground, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1)))

# Create first pendulum body
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(1)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))

# Add visualization to first pendulum (fixed ChFrameD)
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Red
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set initial position (center of mass)
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# Create revolute joint between ground and first pendulum (fixed ChFrameD)
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
sys.AddLink(rev_1)

# Create second pendulum body
pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))

# Add visualization to second pendulum (blue)
cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_2.SetColor(chrono.ChColor(0, 0, 0.6))  # Blue
pend_2.AddVisualShape(cyl_2, chrono.ChFrameD(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set initial position (center of mass at end of first pendulum)
pend_2.SetPos(chrono.ChVector3d(3, 0, 1))

# Create revolute joint between pendulums (fixed ChFrameD)
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFrameD(chrono.ChVector3d(2, 0, 1), chrono.QUNIT))
sys.AddLink(rev_2)

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Demo')  # Updated title
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

    # Log information after 1 second
    if log_info and sys.GetChTime() > 1:
        pos1 = pend_1.GetPos()
        pos2 = pend_2.GetPos()
        print("t = {:.3f}".format(sys.GetChTime()))
        print("Pendulum 1 position: {:.3f} {:.3f} {:.3f}".format(pos1.x, pos1.y, pos1.z))
        print("Pendulum 2 position: {:.3f} {:.3f} {:.3f}".format(pos2.x, pos2.y, pos2.z))
        log_info = False