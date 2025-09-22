import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for moon simulation
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  # Moon gravity

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add visualization sphere to the ground (joint visualization)
sphere_ground = chrono.ChVisualShapeSphere(2)  # Sphere with radius 2
ground.AddVisualShape(sphere_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(2)  # Mass = 2 kg
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Updated inertia

# Add visualization cylinder to the pendulum (modified dimensions)
cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Radius 0.1, height 1.5
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))
# Position cylinder to start at joint (shifted by -0.75 in local X)
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(-0.75, 0, 0), 
                                            chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set initial position and angular velocity
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))
pend_1.SetAngVelParent(chrono.ChVector3d(0, 0, 0.5))  # Initial angular velocity

# Create spherical joint to connect pendulum to ground
sph_1 = chrono.ChLinkLockSpherical()
sph_1.Initialize(ground, pend_1, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), 
                                                   chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(sph_1)

# Create the Irrlicht visualization system
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

    # Log information after 1 second
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()
        print("t = ", sys.GetChTime())
        print("     Position:", pos_1.x, pos_1.y)
        lin_vel_1 = pend_1.GetPosDt()
        print("     Velocity:", lin_vel_1.x, lin_vel_1.y)
        log_info = False