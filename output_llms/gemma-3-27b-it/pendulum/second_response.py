import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  # Moon's gravity

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add a visualization cylinder to the ground
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_PI_2, chrono.ChVector3d(1, 0, 0))))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(2)  # Set the mass of the pendulum (in kg)
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the pendulum
cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Cylinder with radius 0.1 and height 1.5
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the pendulum (center of mass) in the absolute frame
pend_1.SetPos(chrono.ChVector3d(1, 0, 0))

# Set an initial angular velocity for the pendulum
pend_1.SetWVel(chrono.ChVector3d(0, 0, 1))

# Create a spherical joint to connect the pendulum to the ground
spherical_1 = chrono.ChLinkLockSpherical()
spherical_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)), chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(spherical_1)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Pendulum on the Moon')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, -6))
vis.AddTypicalLights()

# Add a visualization sphere for the joint
sph_joint = chrono.ChVisualShapeSphere(0.02)
spherical_1.GetLinkData().AddVisualShape(sph_joint)

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
        print(" ", pos_1.x, " ", pos_1.y)
        lin_vel_1 = pend_1.GetPosDt()
        print(" ", lin_vel_1.x, " ", lin_vel_1.y)
        log_info = False