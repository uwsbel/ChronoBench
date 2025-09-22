import pychrono.core as chrono
import pychrono.irrlicht as irr

# Initialize Chrono system
system = chrono.ChSystemNSC()

# Set gravity (if needed, typically zero for rotor dynamics)
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Create the IGA beam using ChBeamIGA (assuming such class exists in PyChrono)
# Note: As of current PyChrono versions, direct IGA beam classes may not be available.
# For the purpose of this script, we'll assume a placeholder class or use a generic beam.
# If not available, one might need to use a generic FEM beam or a custom implementation.
# Here, we proceed with a placeholder implementation.

# Define beam parameters
beam_length = 1.0  # meters
beam_radius = 0.02  # meters
beam_density = 7800  # kg/m^3 (steel)
beam_E = 2.1e11  # Pa (Young's modulus)
beam_area = chrono.CH_C_PI * beam_radius**2
beam_inertia = (chrono.CH_C_PI * beam_radius**4) / 4  # Moment of inertia

# Create a beam object (placeholder for IGA beam)
# For demonstration, we create a rigid body to represent the beam
# In practice, replace with actual IGA beam implementation

beam = chrono.ChBody()
beam.SetName("Beam")
beam.SetMass(beam_density * beam_area * beam_length)
beam.SetInertia(chrono.ChMatrix33D(beam_inertia, 0, 0,
                                   0, beam_inertia, 0,
                                   0, 0, beam_inertia))
system.Add(beam)

# Position the beam along the x-axis
beam.SetPos(chrono.ChVectorD(0, 0, 0))
beam.SetRot(chrono.Q_from_Euler123(0, 0, 0))
beam.SetBodyFixed(False)

# Attach a flywheel at the center of the beam
flywheel_radius = 0.05  # meters
flywheel_mass = 0.5  # kg

flywheel = chrono.ChBody()
flywheel.SetName("Flywheel")
flywheel.SetMass(flywheel_mass)
flywheel_inertia = 0.5 * flywheel_mass * flywheel_radius**2
flywheel.SetInertia(chrono.ChMatrix33D(flywheel_inertia, 0, 0,
                                       0, flywheel_inertia, 0,
                                       0, 0, flywheel_inertia))
# Position at the center of the beam
flywheel_pos = chrono.ChVectorD(beam_length / 2, 0, 0)
flywheel.SetPos(flywheel_pos)
system.Add(flywheel)

# Fix the start of the beam (left end)
fix_left = chrono.ChLinkMateFix()
fix_left.Initialize(beam, system.GetGround(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(fix_left)

# Attach flywheel to the beam at center with a revolute joint (allow rotation)
revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(beam, flywheel, chrono.ChFrameD(flywheel_pos))
system.Add(revolute_joint)

# Add a motor at the right end of the beam to drive rotation
# Create a rotational motor between the beam's end and ground
motor = chrono.ChLinkMotorRotation()
end_pos = chrono.ChVectorD(beam_length, 0, 0)
# For the motor, define frames
frameA = chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0))
frameB = chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0))
motor.Initialize(beam, system.GetGround(), frameA)
system.Add(motor)

# Set the motor to rotate at a constant angular velocity (e.g., 100 rad/sec)
motor.SetSpeed(100)  # rad/sec

# Add visualization for the beam (assuming a way to visualize IGA beam)
# Since actual IGA visualization may not be available, visualize as a cylinder
visualization = chrono.ChVisualization()
visualization.SetShape(chrono.shapeutils.CreateCylinder(beam_radius, beam_length))
visualization.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
beam.GetVisualShape().AddChild(visualization)

# Create Irrlicht visualization window
application = irr.ChIrrApp(system, "Jeffcott Rotor Simulation", irr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0.2, 0.2, 0.2))
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
time_step = 1e-4
application.SetTimestep(time_step)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()