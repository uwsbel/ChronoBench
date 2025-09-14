import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the moon
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add a visualization sphere for the joint (radius = 2)
sphere_1 = chrono.ChVisualShapeSphere(2)
ground.AddVisualShape(sphere_1, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1)))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(2)  # Mass set to 2 kg
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Updated inertia tensor

# Add visualization cylinder with new dimensions
cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(
    chrono.ChVector3d(0, 0, 0),
    chrono.ChQuaterniond.FromAngleY(chrono.CH_PI_2)
))

# Set initial position and angular velocity
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))
pend_1.SetAngVel(chrono.ChVector3d(0, 0, 2))  # Initial angular velocity (2 rad/s about Z-axis)

# Create spherical joint between ground and pendulum
sph_joint = chrono.ChLinkLockSpherical()
sph_joint.Initialize(ground, pend_1, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1)))
sys.AddLink(sph_joint)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Pendulum Demo')
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

    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()
        print("t = ", sys.GetChTime())
        print("     Position:", pos_1.x, pos_1.y, pos_1.z)
        lin_vel_1 = pend_1.GetPosDt()
        print("     Velocity:", lin_vel_1.x, lin_vel_1.y, lin_vel_1.z)
        log_info = False