import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Change gravity to moon gravity (-1.62 m/s² in Y direction)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))

# Create the ground body
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Change joint visualization to sphere (radius=2)
sphere_joint = chrono.ChVisualShapeSphere(2)  # Sphere for joint visualization
ground.AddVisualShape(sphere_joint, chrono.ChVector3d(0, 0, 1))

# Create pendulum body with modified properties
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(2)  # Mass changed to 2 kg
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Modified inertia

# Set new pendulum position (center at 0.75,0,1 for correct length)
pend_1.SetPos(chrono.ChVector3d(0.75, 0, 1))

# Add visualization cylinder with new dimensions (radius=0.1, height=1.5)
cyl_pend = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Modified dimensions
cyl_pend.SetColor(chrono.ChColor(0.6, 0, 0))
# Rotate cylinder to align with pendulum direction
pend_1.AddVisualShape(cyl_pend, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set initial angular velocity (1 rad/s around Z-axis)
pend_1.SetAngVelLocal(chrono.ChVector3d(0, 0, 1))

# Replace revolute joint with spherical joint
sph_joint = chrono.ChLinkLockSpherical()
sph_joint.Initialize(ground, pend_1, chrono.ChVector3d(0, 0, 1))  # Spherical joint at (0,0,1)
sys.AddLink(sph_joint)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Pendulum Simulation')
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
        print("Position:", pos_1.x, pos_1.y, pos_1.z)
        lin_vel_1 = pend_1.GetPosDt()
        print("Velocity:", lin_vel_1.x, lin_vel_1.y, lin_vel_1.z)
        log_info = False