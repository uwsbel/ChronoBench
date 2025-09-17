import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess

# ---
#  Create the simulation system
# ---

my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ---
#  Create the beam using IGA
# ---

# Define beam parameters
beam_length = 1.0
beam_radius = 0.02
num_segments = 20
beam_young = 2.0e11  # Young's modulus
beam_density = 7850  # Density

# Create the IGA beam
beam = chrono.ChBodyEasy.New(beam_length * 0.5, 0.0, 0.0,  # x, y, z
                             beam_radius, beam_radius, beam_radius,  # rx, ry, rz
                             beam_density)
beam.SetBodyFixed(False)
beam.SetCollide(True)
beam.SetMaterialWithProperties(chrono.ChMaterialSurfaceNSC(), beam_young, 0.3)
beam.SetShapeType(chrono.ChShapeType.SHAPE_IGA_BEAM)

# Set IGA beam parameters
beam.GetShape().Set_Length(beam_length)
beam.GetShape().Set_Radius(beam_radius)
beam.GetShape().Set_Segments(num_segments)
beam.GetShape().Set_Section(chrono.ChIGASectionCircle(beam_radius))

my_system.Add(beam)

# ---
#  Create the flywheel
# ---

flywheel_radius = 0.05
flywheel_mass = 0.1
flywheel = chrono.ChBodyEasy.New(flywheel_radius, flywheel_radius, flywheel_radius,
                                 flywheel_mass, True, True)
flywheel.SetBodyFixed(False)
flywheel.SetCollide(True)
flywheel.SetMaterialWithProperties(chrono.ChMaterialSurfaceNSC(), 2.0e11, 0.3)
flywheel.SetShapeType(chrono.ChShapeType.SHAPE_SPHERE)
flywheel.SetPos(chrono.ChVectorD(beam_length / 2, 0, 0))
my_system.Add(flywheel)

# Connect the flywheel to the beam's center
rev_joint = chrono.ChLink revolute_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(beam, flywheel, chrono.ChCoordsys(chrono.ChVectorD(beam_length / 2, 0, 0)))
my_system.Add(rev_joint)


# ---
#  Create a rotational motor
# ---

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.Set_RotationalSpeed(10)  # rad/s
motor.Set_Torque(1.0)
my_system.Add(motor)

# ---
#  Visualization
# ---

# Create an Irrlicht application to visualize the system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor with IGA Beam')
vis.Initialize()
vis.AddTypicalCamera(chrono.ChVectorD(0, 0, -2))
vis.AddTypicalLights()

# ---
#  Simulation loop
# ---

my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
my_system.SetStepTime(0.001)
time_end = 10.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    my_system.DoStepDynamics(0.001)

    if my_system.GetChTime() > time_end:
        break