import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Parameters
L = 1.0        # Pendulum length (m)
m = 1.0        # Pendulum mass (kg)
r = 0.05       # Bob radius (m)
g = 9.81       # Gravity (m/s²)
theta0 = 30    # Initial angle (degrees)
time_step = 0.001  # Simulation time step (s)
log_interval = 0.1 # Logging interval (seconds)

# Convert initial angle to radians
theta0_rad = theta0 * chrono.CH_DEG_TO_RAD

# Create Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -g, 0))

# Create ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetName("Ground")
system.AddBody(ground)

# Add visualization assets to ground
box_asset = chrono.ChBoxShape(0.2, 0.1, 0.2)
box_asset.SetColor(chrono.ChColor(0.6, 0.3, 0.0))
ground.AddVisualShape(box_asset, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

# Create pendulum bob (sphere)
bob = chrono.ChBody()
bob.SetMass(m)
inertia = chrono.ChMatrix33D()
inertia.FillDiagonal(0.4 * m * r * r, 0.4 * m * r * r, 0.4 * m * r * r)
bob.SetInertia(inertia)

# Set initial position and velocity
bob.SetPos(chrono.ChVectorD(L * math.sin(theta0_rad), -L * math.cos(theta0_rad), 0))
bob.SetRot(chrono.Q_from_AngZ(theta0_rad))
bob.SetName("Bob")
system.AddBody(bob)

# Add visualization assets to pendulum
sphere_asset = chrono.ChSphereShape(r)
sphere_asset.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
bob.AddVisualShape(sphere_asset)

rod_asset = chrono.ChCylinderShape()
rod_asset.GetCylinderGeometry().rad = 0.015
rod_asset.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
rod_asset.GetCylinderGeometry().p2 = chrono.ChVectorD(0, L, 0)
rod_asset.SetColor(chrono.ChColor(0.7, 0.7, 0.7))
bob.AddVisualShape(rod_asset)

# Create revolute joint between ground and pendulum
pivot = chrono.ChLinkRevolute()
pivot.Initialize(ground, bob, 
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.AddLink(pivot)

# Create Irrlicht visualization
application = chronoirr.ChIrrApp(system, "Simple Pendulum Simulation", chronoirr.dimension2du(1280, 720))
application.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(1.5, -0.5, 1.5))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop setup
application.SetTimestep(time_step)
last_log_time = 0

# Main simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.Render()
    application.DoStep()
    application.EndScene()
    
    # Log pendulum state at specified intervals
    current_time = system.GetChTime()
    if current_time - last_log_time >= log_interval:
        pos = bob.GetPos()
        vel = bob.GetPos_dt()
        print(f"Time: {current_time:.3f} s | "
              f"Position: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}) m | "
              f"Velocity: ({vel.x:.4f}, {vel.y:.4f}, {vel.z:.4f}) m/s")
        last_log_time = current_time