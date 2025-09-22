import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fem as fem

# Initialize the Chrono::Engine
chrono.SetChronoDataPath('../data/')

# Create a Chrono::Engine system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create the IGA beam
beam = fem.ChElementBeamIGA()
beam.SetN(10)  # Number of control points along the beam
beam.SetP(3)   # Polynomial degree

# Define the beam's material properties
beam.SetYoungModulus(210e9)  # Young's modulus (Pa)
beam.SetDensity(7800)        # Density (kg/m^3)
beam.SetCrossArea(0.01)      # Cross-sectional area (m^2)
beam.SetAsymmetry(0.5)       # Asymmetry factor (dimensionless)

# Define the beam's geometry
beam.SetLength(1.0)          # Length of the beam (m)

# Add the beam to the system
system.Add(beam)

# Create a flywheel and attach it to the center of the beam
flywheel = chrono.ChBodyEasyCylinder(0.1, 0.05, 1000, True, True)
flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))  # Position at the center of the beam
system.Add(flywheel)

# Create a rotational motor to drive one end of the beam
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam.GetMarker('END'), flywheel.GetMarker('END'))
motor.SetMotorFunction(chrono.ChFunction_Const(chrono.ChVectorD(10)))  # Constant speed of 10 rad/s
system.Add(motor)

# Create an Irrlicht visualization
visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle('Jeffcott Rotor Simulation')
visualization.Initialize()
visualization.AddLogo()
visualization.AddSkyBox()
visualization.AddTypicalLights()
visualization.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))

# Create FEM visualization
fem_visualization = fem.ChVisualizationFEMmesh()
fem_visualization.SetMeshType(fem.ChVisualizationFEMmesh.E_LESS_LINES)
fem_visualization.SetSmoothFaces(True)
fem_visualization.SetWireframe(True)
fem_visualization.SetDrawContacts(True)
fem_visualization.SetDrawBeams(True)
fem_visualization.SetDrawElements(True)
fem_visualization.SetDrawNodes(True)
fem_visualization.SetColorScaleMin(0)
fem_visualization.SetColorScaleMax(1)
beam.AddVisualization(fem_visualization)

# Run the simulation
system.SetSolverType(chrono.ChSolver.MINRES)
system.SetMaxPenetrationRecoverySpeed(1.0)
system.SetTimestepperType(chrono.ChTimestepper.HHT)
system.SetLcpSolverType(chrono.ChSolver.PARDISO)
system.SetMaxIterationsLcp(100)
system.SetMaxPenetrationRecoverySpeed(0.1)
system.SetLcpSolverMaxIterations(100)

# Simulation parameters
step_size = 0.01
total_time = 10.0

# Simulation loop
while visualization.Run():
    system.DoStepDynamics(step_size)
    visualization.Render()

# Clean up
visualization.Close()