import chrono as ch
import chrono.irrlicht as irr
import chrono.fem as fem
import math

# Initialize PyChrono environment
ch.CH_init()

# Create the system
system = ch.ChSystem()

# Create the IGA beam
beam = fem.ChBeamIGA()
beam.SetNodesSpacing(0.1)  # Adjust spacing as needed
beam.SetLength(1.0)      # Adjust length as needed
beam.SetSection(fem.ChBeamSectionIGA(0.05, 0.01, 0.01))  # Adjust section dimensions as needed
beam.SetMaterial(fem.ChFEMMaterial(200e9, 0.3, 7850))  # Steel, adjust as needed
system.Add(beam)

# Create a flywheel
flywheel = ch.ChBodyEasyBody(1.0,  # mass
                              0.5,  # radius
                              ch.ChFrame(ch.Vector(0, 0, 0.5), ch.Quaternion(1, 0, 0, 0)))
flywheel.SetPos(ch.Vector(0.5, 0, 0.5))  # Position at beam center
system.Add(flywheel)

# Attach flywheel to beam
ch.ChLinkMateFix().Connect(beam, flywheel)

# Create a rotational motor
motor = ch.ChLinkMotorRotation()
motor.Init(beam, 
           ch.Frame(ch.Vector(0, 0, 0), ch.Quaternion(1, 0, 0, 0)), 
           ch.Frame(ch.Vector(0, 0, 0), ch.Quaternion(1, 0, 0, 0)))
motor.SetMotorFunction(ch.ChFunction_Sine(0, 1, 0))  # Sine function for rotation
system.Add(motor)

# Create Irrlicht application
app = irr.ChIrrApp(system, 'Jeffcott Rotor Simulation', 800, 600)

# Add FEM visualization
fem_vis = ch.ChFEMVisualization(beam)
fem_vis.SetFEMdataResolution(4)
app.AddVisualization(fem_vis)

# Set visualization parameters
app.SetTimestep(0.01)
app.SetCollision(True)

# Run the simulation
app.Run()