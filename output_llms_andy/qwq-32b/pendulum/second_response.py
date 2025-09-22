import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2) for the Moon
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -1.62, 0))

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add a visualization cylinder to the ground
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Radius 0.2, height 0.4
ground.AddVisualShape(cyl_ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(2)  # 2 kg mass
pend_1.SetInertiaXX(chrono.ChVectorD(0.4, 1.5, 1.5))  # New inertia tensor

# Add a visualization cylinder to the pendulum with adjusted dimensions
cyl_pend = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Radius 0.1, height 1.5
cyl_pend.SetColor(chrono.ChColor(0.6, 0, 0))
# Rotate cylinder along Y-axis by 90 degrees to align with X-axis
pend_1.AddVisualShape(cyl_pend, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngY(chrono.CH_PI_2)))

# Set initial position (center of mass at (1,0,1))
pend_1.SetPos(chrono.ChVectorD(1, 0, 1))

# Set initial angular velocity (around Z-axis)
pend_1.SetWvel_par(chrono.ChVectorD(0, 0, 5))  # 5 rad/s

# Create a spherical joint to connect the pendulum to the ground
sph_1 = chrono.ChLinkLockSpherical()
# Ground's frame at (0,0,1), pendulum's offset from COM is (-1,0,0) to reach joint position (0,0,1)
sph_1.Initialize(ground, pend_1, 
                chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)),  # Ground frame
                chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0)))  # Pendulum frame (offset from COM)
sys.AddLink(sph_1)

# Add visualization sphere for the joint
sphere_vis = chrono.ChVisualShapeSphere(2)  # Radius 2
sphere_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
sph_1.AddVisualShape(sphere_vis)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Pendulum Simulation')
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
    sys.DoStepDynamics(1e-3)

    # Log information after 1 second of simulation time
    if log_info and sys.GetChTime() > 1:
        pos = pend_1.GetPos()
        print(f"t = {sys.GetChTime():.2f}")
        print(f"Position: {pos.x:.2f}, {pos.y:.2f}")
        lin_vel = pend_1.GetPos_dt()
        print(f"Linear Velocity: {lin_vel.x:.2f}, {lin_vel.y:.2f}")
        log_info = False

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2) for the Moon
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -1.62, 0))

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add a visualization cylinder to the ground
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Radius 0.2, height 0.4
ground.AddVisualShape(cyl_ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(2)  # 2 kg mass
pend_1.SetInertiaXX(chrono.ChVectorD(0.4, 1.5, 1.5))  # New inertia tensor

# Add a visualization cylinder to the pendulum with adjusted dimensions
cyl_pend = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Radius 0.1, height 1.5
cyl_pend.SetColor(chrono.ChColor(0.6, 0, 0))
# Rotate cylinder along Y-axis by 90 degrees to align with X-axis
pend_1.AddVisualShape(cyl_pend, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngY(chrono.CH_PI_2)))

# Set initial position (center of mass at (1,0,1))
pend_1.SetPos(chrono.ChVectorD(1, 0, 1))

# Set initial angular velocity (around Z-axis)
pend_1.SetWvel_par(chrono.ChVectorD(0, 0, 5))  # 5 rad/s

# Create a spherical joint to connect the pendulum to the ground
sph_1 = chrono.ChLinkLockSpherical()
# Ground's frame at (0,0,1), pendulum's offset from COM is (-1,0,0) to reach joint position (0,0,1)
sph_1.Initialize(ground, pend_1, 
                chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)),  # Ground frame
                chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0)))  # Pendulum frame (offset from COM)
sys.AddLink(sph_1)

# Add visualization sphere for the joint
sphere_vis = chrono.ChVisualShapeSphere(2)  # Radius 2
sphere_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
sph_1.AddVisualShape(sphere_vis)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Pendulum Simulation')
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
    sys.DoStepDynamics(1e-3)

    # Log information after 1 second of simulation time
    if log_info and sys.GetChTime() > 1:
        pos = pend_1.GetPos()
        print(f"t = {sys.GetChTime():.2f}")
        print(f"Position: {pos.x:.2f}, {pos.y:.2f}")
        lin_vel = pend_1.GetPos_dt()
        print(f"Linear Velocity: {lin_vel.x:.2f}, {lin_vel.y:.2f}")
        log_info = False