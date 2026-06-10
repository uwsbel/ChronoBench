# =============================================================================
# PyChrono Mass-Spring-Damper System using ChLinkTSDA
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# =============================================================================
# System Initialization
# =============================================================================

# Create the Chrono physical system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# =============================================================================
# Material and Visual Setup
# =============================================================================

# Create a shared contact material
contact_material = chrono.ChContactMaterialNSC()
contact_material.SetFriction(0.5)

# =============================================================================
# Ground Body
# =============================================================================

# Create the ground body (fixed)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
system.AddBody(ground)

# Add visual shape to ground (flat box)
ground_shape = chrono.ChVisualShapeBox(4.0, 0.2, 4.0)
ground_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(ground_shape, chrono.ChFramed(chrono.ChVector3d(0, -0.1, 0)))

# Add a cylinder to ground at attachment point
ground_attach_shape = chrono.ChVisualShapeCylinder(0.05, 0.3)
ground_attach_shape.SetColor(chrono.ChColor(0.6, 0.3, 0.1))
ground.AddVisualShape(ground_attach_shape, 
                      chrono.ChFramed(chrono.ChVector3d(0, 2.0, 0),
                                      chrono.QuatFromAngleX(math.pi / 2)))

# =============================================================================
# Mass Body
# =============================================================================

# Define mass properties
mass_value = 2.0         # kg
spring_rest_length = 1.5 # m
initial_offset = 0.5     # m (displacement from rest)

# Create the mass body
mass_body = chrono.ChBody()
mass_body.SetMass(mass_value)
mass_body.SetPos(chrono.ChVector3d(0, spring_rest_length + initial_offset, 0))

# Set inertia tensor for a cube-like shape
side = 0.3
inertia = (1.0 / 6.0) * mass_value * side * side
mass_body.SetInertiaXX(chrono.ChVector3d(inertia, inertia, inertia))

system.AddBody(mass_body)

# Add visual shape to mass (box)
mass_shape = chrono.ChVisualShapeBox(side, side, side)
mass_shape.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
mass_body.AddVisualShape(mass_shape)

# =============================================================================
# Spring-Damper Connection (ChLinkTSDA)
# =============================================================================

# Define spring-damper parameters
spring_coeff   = 50.0   # N/m  - spring stiffness
damping_coeff  = 5.0    # N·s/m - damping coefficient
rest_length    = spring_rest_length  # natural length of the spring

# Attachment points
ground_attach_point = chrono.ChVector3d(0, 2.0, 0)   # point on ground (top)
mass_attach_point   = chrono.ChVector3d(0, 0, 0)      # center of mass body

# Create the TSDA (Translational Spring-Damper-Actuator) link
spring_damper = chrono.ChLinkTSDA()
spring_damper.SetSpringCoefficient(spring_coeff)
spring_damper.SetDampingCoefficient(damping_coeff)
spring_damper.SetRestLength(rest_length)

# Initialize: body1=ground, body2=mass, positions in absolute frame
spring_damper.Initialize(
    ground,                    # Body 1 (ground)
    mass_body,                 # Body 2 (mass)
    True,                      # Use absolute frame positions
    ground_attach_point,       # Attachment point on body 1
    mass_body.GetPos()         # Attachment point on body 2
)

system.AddLink(spring_damper)

# Add visual shape for the spring (coil/segment visualization)
spring_visual = chrono.ChVisualShapeSpring(0.1, 80, 15)
spring_visual.SetColor(chrono.ChColor(0.9, 0.2, 0.2))
spring_damper.AddVisualShape(spring_visual)

# =============================================================================
# Irrlicht Visualization
# =============================================================================

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Mass-Spring-Damper System (ChLinkTSDA)")
vis.Initialize()

# Add a logo and sky box
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()

# Configure camera: position, look-at target, up vector
vis.AddCamera(
    chrono.ChVector3d(3.0, 3.0, 5.0),   # Camera position
    chrono.ChVector3d(0.0, 2.0, 0.0)    # Look-at target
)

# Add lighting
vis.AddTypicalLights()
vis.AddLight(
    chrono.ChVector3d(4.0, 8.0, 4.0),   # Light position
    10.0,                                 # Radius / intensity
    chrono.ChColor(1.0, 1.0, 1.0)        # White light
)

# =============================================================================
# Simulation Loop
# =============================================================================

# Time-step settings
time_step = 0.001   # seconds
end_time  = 10.0    # seconds

# Print header for data output
print("=" * 60)
print(f"{'Time':>8}  {'Pos_Y':>10}  {'Vel_Y':>10}  {'Spring_F':>12}")
print("=" * 60)

# Simulation loop
time = 0.0
frame_count = 0

while vis.Run() and time < end_time:
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Print data every 0.1 seconds
    if frame_count % 100 == 0:
        pos_y    = mass_body.GetPos().y
        vel_y    = mass_body.GetPosDt().y
        spring_f = spring_damper.GetForce()
        print(f"{time:>8.3f}  {pos_y:>10.4f}  {vel_y:>10.4f}  {spring_f:>12.4f}")

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)

    time += time_step
    frame_count += 1

print("=" * 60)
print("Simulation complete.")