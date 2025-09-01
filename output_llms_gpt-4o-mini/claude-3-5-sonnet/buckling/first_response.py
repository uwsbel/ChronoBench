import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

# Initialize the ChSystem
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a mesh, that is a container for groups of elements and their referenced nodes.
mesh = fea.ChMesh()

# Create beam material
beam_material = fea.ChElasticityKirchhoffMaterial()
beam_material.SetYoungModulus(2.1e11)  # Steel
beam_material.SetPoissionRatio(0.3)
beam_material.SetDensity(7850)

# Create beam section
beam_section = fea.ChBeamSectionEulerAdvanced()
beam_width = 0.05
beam_height = 0.1
beam_section.SetAsRectangularSection(beam_width, beam_height)
beam_section.SetYoungModulus(beam_material.GetYoungModulus())
beam_section.SetGshearModulus(beam_material.GetGshearModulus())
beam_section.SetDensity(beam_material.GetDensity())

# Create beam elements
beam_length = 2.0
n_elements = 20
dx = beam_length / n_elements

for i in range(n_elements):
    node_a = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * dx, 0, 0)))
    node_b = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD((i + 1) * dx, 0, 0)))
    element = fea.ChElementBeamEuler()
    element.SetNodes(node_a, node_b)
    element.SetSection(beam_section)
    mesh.AddElement(element)

    if i == 0:
        node_a.SetFixed(True)

# Add the mesh to the system
system.Add(mesh)

# Create ground
ground = chrono.ChBodyEasyBox(5, 0.2, 1, 1000, True)
ground.SetBodyFixed(True)
system.Add(ground)

# Create a motor at the end of the beam
motor = chrono.ChLinkMotorLinearPosition()
motor.Initialize(
    mesh.GetNode(n_elements),
    ground,
    chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0))
)

# Custom motor function
class MyMotorFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()
    def Get_y(self, x):
        return 0.1 * math.sin(0.5 * x)  # Sinusoidal motion

motor.SetMotionFunction(MyMotorFunction())
system.AddLink(motor)

# Create the Irrlicht visualization
application = chronoirr.ChIrrApp(system, "Beam Buckling Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 1, -2))

# Enable drawing of FEA mesh
application.AssetBindAll()
application.AssetUpdateAll()

# Set up the solver
solver = chrono.ChSolverMINRES()
system.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableWarmStart(True)

# Set up the timestepper
stepper = chrono.ChTimestepperHHT(system)
stepper.SetStepControl(True)
stepper.SetMaximuTimeStep(0.01)
stepper.SetVerbose(True)
system.SetTimestepper(stepper)

# Simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    # Update the system
    application.DoStep()
    
    # Draw additional information
    chronoirr.drawGrid(application.GetVideoDriver(), 0.1, 0.1, 
                       20, 20, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)),
                       chronoirr.SColor(255, 80, 100, 100), True)
    
    application.EndScene()