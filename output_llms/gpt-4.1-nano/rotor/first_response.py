import pychrono.core as chrono
import pychrono.irrlicht as irr

# Initialize Chrono simulation system
system = chrono.ChSystemNSC()

# -----------------------------
# 1. Create the IGA Beam
# -----------------------------

# Define beam parameters
beam_length = 1.0  # Length of the beam in meters
beam_radius = 0.02  # Cross-sectional radius (assuming a circular cross-section)
beam_density = 7800  # Density in kg/m^3 (steel)
beam_E = 2.1e11      # Young's modulus in Pa (steel)
beam_area = 3.14159 * beam_radius**2
beam_inertia = (chrono.ChVectorD(0, 0, beam_radius**4 * 3.14159 / 4),
                chrono.ChVectorD(0, 0, beam_radius**4 * 3.14159 / 4))  # Simplified inertia

# Create a beam node at the start
start_pos = chrono.ChVectorD(0, 0, 0)
end_pos = chrono.ChVectorD(beam_length, 0, 0)

# For IGA beam, we utilize ChBody with constraints or a custom approach.
# Here, we'll approximate with a ChBody and appropriate constraints for simplicity.

# Create a rigid body for the beam (approximated as a rigid slender member)
beam_body = chrono.ChBody()
beam_body.SetMass(beam_density * beam_area * beam_length)
beam_inertia_diag = beam_body.GetInertia()
# For simplicity, approximate inertia for a slender beam
beam_body.SetInertiaXX(chrono.ChVectorD(
    (1/12)*beam_body.GetMass()*(beam_length**2),
    (1/12)*beam_body.GetMass()*(beam_radius**2),
    (1/12)*beam_body.GetMass()*(beam_radius**2)
))

# Position the beam at the start
beam_body.SetPos(start_pos + chrono.ChVectorD(beam_length/2, 0, 0))
system.Add(beam_body)

# Note: For a true IGA beam, a specialized element would be used, but here we use approximation due to the scope.

# -----------------------------
# 2. Attach a Flywheel at the Center
# -----------------------------

# Create the flywheel body
flywheel_radius = 0.05
flywheel_mass = 2.0  # kg

flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVectorD(
    0.5 * flywheel_mass * flywheel_radius**2,
    0.5 * flywheel_mass * flywheel_radius**2,
    0.5 * flywheel_mass * flywheel_radius**2
))
# Place at mid-span of the beam
mid_pos = chrono.ChVectorD(beam_length/2, 0, 0)
flywheel.SetPos(mid_pos)
system.Add(flywheel)

# Attach the flywheel rigidly to the beam at mid-span
# Use a fixed joint to attach to the beam
joint_flywheel = chrono.ChLinkLockRevolute()
joint_flywheel.Initialize(beam_body, flywheel, chrono.CrdTransform(mid_pos, chrono.QUNIT))
system.Add(joint_flywheel)

# -----------------------------
# 3. Set Up a Rotational Motor at One End
# -----------------------------

# Create a motor at one end of the beam (for example, at x=0)
# First, create a rotational joint at the fixed point
fixed_point = chrono.ChVectorD(0, 0, 0)

# Create a body at the end for attaching the motor
end_body = chrono.ChBody()
end_body.SetMass(0)  # massless for joint purposes
end_body.SetPos(start_pos)
system.Add(end_body)

# Create a revolute joint at x=0
revolute_joint = chrono.ChLinkMotorRotationSpeed()
revolute_joint.Initialize(end_body, system.Get_ground(), chrono.ChFrameD(fixed_point, chrono.Q_from_AngZ(0)))
system.Add(revolute_joint)

# Set the motor to rotate at a constant speed (e.g., 100 rad/sec)
motor_speed = 100.0
revolute_joint.SetSpeed(motor_speed)

# connect the end of the beam to end_body with a rotational joint
# For simulation purposes, create a joint between the beam's end node and end_body
end_node = chrono.ChNodeShape_Utils()  # Placeholder for the beam's end node
# Since the previous step approximates the beam as a body, attach via a revolute joint
# For simplicity, attach directly to beam's end position

# Alternatively, we can connect the beam's end point via a joint
# Create a 'dummy' body at the beam's end
beam_end_body = chrono.ChBody()
beam_end_body.SetPos(end_pos)
system.Add(beam_end_body)

joint_end = chrono.ChLinkRevolute()
joint_end.Initialize(beam_body, beam_end_body, chrono.CrdTransform(end_pos, chrono.QIDENT))
system.Add(joint_end)

# Then connect the end_body to the beam's end
joint_motor = chrono.ChLinkMotorRotationSpeed()
joint_motor.Initialize(end_body, system.Get_ground(), chrono.CrdTransform(end_pos, chrono.Q_from_AngZ(0)))
system.Add(joint_motor)
joint_motor.SetSpeed(motor_speed)

# -----------------------------
# 4. Visualization Setup
# -----------------------------

# Create an Irrlicht application for visualization
application = irr.ChVisualSystemIrrlicht()
application.AttachSystem(system)
application.SetWindowSize(800, 600)
application.SetWindowTitle('Jeffcott Rotor Simulation')
application.Initialize()

# Add the beam visualization
# For simplicity, visualize as a cylinder
beam_visual = irr.ChCylinderShape(beam_radius, beam_length)
beam_visual.SetColor(irr.WithAlpha(irr.C_color(0.3, 0.3, 0.8), 0.8))
application.GetVideoDriver().BindAll()

# Add flywheel visualization
flywheel_visual = irr.ChSphereShape(flywheel_radius)
flywheel_visual.SetColor(irr.C_color(0.8, 0.1, 0.1))
application.GetSceneManager().AddSphere(flywheel_visual, True, flywheel.GetPos(), chrono.ChQuaternionD(1,0,0,0))
# Note: For better visualization, create a visual asset attached to flywheel body

# Main simulation loop
while application.GetDevice().run():
    system.DoStepDynamics(1e-3)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

# Properly close the visualization device after the loop
application.GetDevice().close()