import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

# --- Simulation setup ---
chrono.SetChronoDataPath("../chrono_data/")  # Adjust path if needed

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# --- Beam parameters ---
beam_length = 1.0  # meters
beam_height = 0.1  # meters
beam_width = 0.05  # meters
beam_density = 7850  # kg/m^3
youngs_modulus = 2e11  # Pa
poissons_ratio = 0.3

# --- Create the beam using FEA ---
beam = fea.ChBeamFiniteElement(beam_length, beam_height, beam_width, beam_density, youngs_modulus, poissons_ratio)
beam.MapMaterialProperties(youngs_modulus, poissons_ratio)
beam.SetYoungModulus(youngs_modulus)
beam.SetPoissonRatio(poissons_ratio)

# Add the beam to the system
system.Add(beam)

# --- Fix one end of the beam ---
beam.Set_Fix_X(True)
beam.Set_Fix_Y(True)
beam.Set_Fix_Z(True)
beam.Set_Fix_WX(True)
beam.Set_Fix_WY(False)  # Allow rotation around Y-axis
beam.Set_Fix_WZ(True)

# --- Apply a vertical force at the free end ---
force_magnitude = 100  # Newtons
force_point = chrono.ChVectorD(beam_length, 0, 0)
system.Add_Force(force_point, chrono.ChVectorD(0, -force_magnitude, 0))

# --- Custom Motor (optional - for more complex control) ---
# Define a custom motor function to apply a time-varying force
class CustomMotor:
    def __init__(self, beam, force_point, magnitude):
        self.beam = beam
        self.force_point = force_point
        self.magnitude = magnitude
        self.time = 0.0

    def ApplyForce(self, time):
        self.time = time
        force_value = self.magnitude * chrono.ChMath.sin(2 * chrono.CH_C_PI * time)  # Example: sinusoidal force
        force = chrono.ChVectorD(0, -force_value, 0)
        self.beam.ApplyForce(self.force_point, force)

# Uncomment to use the custom motor
# custom_motor = CustomMotor(beam, force_point, force_magnitude)
# system.RegisterForce(custom_motor.ApplyForce)


# --- Solver and Timestepper settings ---
system.SetSolverType(chrono.ChSolver.Type_SOR)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)
system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
system.SetStepTime(0.001)

# --- Visualization ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(beam_length * 1.5, beam_height * 2, -beam_length * 1.5))
vis.AddTypicalLights()

# --- Simulation loop ---
time_end = 5.0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(0.001)

    # If using custom motor, uncomment this line
    # custom_motor.ApplyForce(system.GetChTime())

    if system.GetChTime() > time_end:
        break

vis.GetIrrlichtApplication().Close()