import pychrono as chrono
import pychrono.irrlicht as chronoirr
import os

# ---------------------------------------------------------------------
# Set the path to the Chrono data files (for models, textures, etc.)
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'chrono_data', ''))
# For Conda users, a more robust way might be:
# import pychrono.tools.paths as p
# chrono.SetChronoDataPath(os.path.join(p.get_chrono_data_path(), ''))


# ---------------------------------------------------------------------
# Create the Chrono physical system
# ---------------------------------------------------------------------
system = chrono.ChSystemNSC()  # Use NSC (non-smooth contact) solver, common for general MBS

# Set gravity
system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

# ---------------------------------------------------------------------
# Create the ground body
# ---------------------------------------------------------------------
ground = chrono.ChBody()
ground.SetBodyFixed(True)  # Ground is static
ground.SetPos(chrono.ChVectorD(0, 0, 0)) # Position at origin
system.Add(ground)

# Add a small visual marker for the ground anchor point (optional)
ground_marker = chrono.ChSphereShape(0.05)
ground_marker.SetColor(chrono.ChColor(0.6, 0.6, 0.6)) # Grey color
ground.AddVisualShape(ground_marker, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0))) # Anchor point slightly above ground origin

# ---------------------------------------------------------------------
# Create the mass body
# ---------------------------------------------------------------------
mass_val = 1.0  # kg
mass_radius = 0.2 # m
initial_mass_pos = chrono.ChVectorD(0, 1.5, 0) # Initial position above ground anchor

mass = chrono.ChBody()
mass.SetMass(mass_val)
mass.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01)) # Basic inertia (exact value not critical for this demo)
mass.SetPos(initial_mass_pos)
system.Add(mass)

# Add a visual shape for the mass
mass_shape = chrono.ChSphereShape(mass_radius)
mass_shape.SetColor(chrono.ChColor(0.3, 0.4, 0.8)) # Blue-ish color
mass.AddVisualShape(mass_shape)

# ---------------------------------------------------------------------
# Create the spring-damper (ChLinkTSDA)
# ---------------------------------------------------------------------
# Connection points:
# Point 1: On the mass body (local coordinates relative to mass's center)
pt1_mass_local = chrono.ChVectorD(0, 0, 0)
# Point 2: On the ground body (local coordinates relative to ground's center)
# We want to connect to the visual marker we placed at (0, 0.5, 0) in global.
# Since ground is at (0,0,0) and not rotated, this is also (0, 0.5, 0) in ground's local frame.
pt2_ground_local = chrono.ChVectorD(0, 0.5, 0)

# Default linear spring-damper parameters
spring_stiffness = 100.0  # N/m
damping_coefficient = 5.0  # Ns/m
# The spring will be initially stretched because mass_pos.y (1.5) - ground_anchor.y (0.5) = 1.0
# Let's set a rest length different from the initial length to see oscillation.
rest_length = 0.7 # m

# Create the TSDA link
tsda = chrono.ChLinkTSDA()
tsda.Initialize(mass,              # First body
                ground,            # Second body
                False,             # False: positions are in local COG frames of bodies
                pt1_mass_local,    # Attachment point on first body (local)
                pt2_ground_local)  # Attachment point on second body (local)

tsda.SetSpringCoefficient(spring_stiffness)
tsda.SetDampingCoefficient(damping_coefficient)
tsda.SetRestLength(rest_length)
system.Add(tsda)

# Add a visual shape for the spring
spring_shape = chrono.ChSpringShape(0.03, 80, 15) # radius, resolution, num_turns
spring_shape.SetColor(chrono.ChColor(0.8, 0.8, 0.2)) # Yellow-ish color
tsda.AddVisualShape(spring_shape)


# ---------------------------------------------------------------------
# Create the Irrlicht visualization
# ---------------------------------------------------------------------
myapplication = chronoirr.ChIrrApp(system, "Mass-Spring-Damper Demo", chronoirr.dimension2du(1024, 768))
myapplication.AddTypicalLights()
myapplication.AddTypicalSky()
myapplication.AddTypicalCamera(chronoirr.vector3df(1.5, 1.5, -2.5), # Camera position
                               chronoirr.vector3df(0, 0.8, 0))    # Look at point

# Bind all visual shapes to their assets
myapplication.AssetBindAll()
myapplication.AssetUpdateAll()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
myapplication.SetTimestep(0.005) # s
myapplication.SetTryRealtime(True)

print("Starting simulation... Press Esc to exit.")

while myapplication.GetDevice().run():
    myapplication.BeginScene(True, True, chronoirr.SColor(255, 140, 160, 190)) # Background color
    myapplication.DrawAll()
    
    # Print some info every few steps
    if system.GetChTime() % 0.1 < myapplication.GetTimestep(): # Approximately every 0.1s
        print(f"Time: {system.GetChTime():.2f} s, Mass Y: {mass.GetPos().y:.3f} m, Spring Length: {tsda.GetLength():.3f} m")

    myapplication.DoStep()
    myapplication.EndScene()

print("Simulation finished.")

import pychrono.tools.paths as p
    chrono.SetChronoDataPath(os.path.join(p.get_chrono_data_path(), ''))