import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# -----------------------------------------------------------------------------
# 1. Initialize the PyChrono environment and core components
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # Use default data path

sys = chrono.ChSystemSMC()

# Use a suitable timestepper and solver for nonlinear dynamic analysis
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
sys.SetSolver(solver)

sys.SetTimestepperType(chrono.ChTimestepper.Type.ALMANSS)
timestepper = sys.GetTimestepper()
timestepper.SetStepControl(True)
timestepper.SetMinStep(1e-5)
timestepper.SetMaxStep(1e-3)
timestepper.SetTolerance(1e-5)

sys.SetTimeStep(1e-4)

# -----------------------------------------------------------------------------
# 2. Create the finite element mesh and beam for buckling scenario
# -----------------------------------------------------------------------------
mesh = fea.ChMesh()

# Beam geometry parameters
length = 1.0     # 1 meter long
num_elements = 20
element_length = length / num_elements

# Material properties (steel-like)
E = 2.1e11        # Young's modulus (Pa)
nu = 0.3          # Poisson ratio
rho = 7800        # Density (kg/m^3)
g = 9.81          # gravity (m/s^2)

# Section properties - assume rectangular cross section
beam_width = 0.02  # 2 cm
beam_height = 0.02 # 2 cm
A = beam_width * beam_height
Iyy = (beam_width * beam_height ** 3) / 12.0

# Create the beam section and material
beam_section = fea.ChBeamSectionAdvanced()
beam_section.SetYoungModulus(E)
beam_section.SetGwithPoissonRatio(E, nu)  # approximate shear modulus G given nu
beam_section.SetDensity(rho)
beam_section.SetAsRectangularSection(beam_width, beam_height)
beam_section.SetBeamRalstonShear(True)

# Create the nodes for the beam (line of nodes)
nodes = []
for i in range(num_elements + 1):
    x = i * element_length
    node = fea.ChNodeFEAxyzrot(chrono.ChVectorD(x, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)

# Create the beam elements connecting the nodes
for i in range(num_elements):
    beam_element = fea.ChElementBeamEuler()
    beam_element.SetNodes(nodes[i], nodes[i + 1])
    beam_element.SetSection(beam_section)
    beam_element.SetPitchNeutralAxis(True)
    mesh.AddElement(beam_element)

# Add gravity to the system
sys.Set_G_acc(chrono.ChVectorD(0, -g, 0))

# Add the mesh to the system
sys.Add(mesh)

# -----------------------------------------------------------------------------
# 3. Fix the left end of the beam (fully fixed constraint)
# -----------------------------------------------------------------------------
left_node = nodes[0]

# Fix position (XYZ) and rotation (ROT)
left_node.SetFixed(True)

# -----------------------------------------------------------------------------
# 4. Add a custom motor at the right end applying axial compression load
# -----------------------------------------------------------------------------
# We'll constrain the last node to a motor moving in X direction pushing inward

right_node = nodes[-1]

# Create a link motor between right_node and ground
motor_frame = chrono.ChFrameD()

# Initialize motor frame at right node position, aligned with X direction
motor_frame.SetPos(right_node.GetPos())

motor = chrono.ChLinkMotorRotationAngle()
motor.SetMotorFunction(chrono.ChFunction_Ramp(-0.01, 0))  
# Here -0.01 rad/s starting at t=0 (we will override this by a custom function below)
# Actually since it is beam axial compression, a rotational motor is not ideal.
# Instead, use a linear motor (translation) along X axis.

# Actually use ChLinkMotorLinearPosition for translation control

motor_lin = chrono.ChLinkMotorLinearPosition()
motor_lin.SetName("AxialCompressionMotor")

motor_lin.Initialize(right_node, sys.GetGroundBody(), chrono.ChFrameD(right_node.GetPos()))

# Define a custom motor function applying a slow displacement inward along X:

class AxialCompressionFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()
    def Get_y(self, x):
        # x: time
        # Returns the position command along X (displacement)
        # Ramp from 0 to -0.01 m over 1 second, then hold
        if x < 1.0:
            return -0.01 * x
        else:
            return -0.01
    def Clone(self):
        return AxialCompressionFunction()

motor_lin.SetMotorFunction(AxialCompressionFunction())

sys.Add(motor_lin)

# To ensure beam element connectivity and constraints are respected, add nodes constraints:

# Optionally, to keep the right node rotationally free but constrained in other directions,
# fix Y and Z translations:
right_node.SetFixed(False)  # Let motor move it in X
# But limit Y,Z translations by constraints - PyChrono does not offer per-axis fixed easily on nodes,
# So alternatively constrain via links:

# Create a prismatic joint to block Y and Z movement, allow X translation:
prismatic = chrono.ChLinkLockPrismaticX()
prismatic.Initialize(right_node, sys.GetGroundBody(), chrono.ChFrameD(right_node.GetPos()))
sys.Add(prismatic)

# -----------------------------------------------------------------------------
# 5. Set solver parameters for better accuracy and robustness (already done above)
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# 6. Visualization: Use Irrlicht application to visualize beam and simulation
# -----------------------------------------------------------------------------
application = chronoirr.ChIrrApp(sys, "Beam Buckling Simulation with FEA and Custom Motor", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0.5, 0.3, 1.5))
application.SetTimestep(1e-4)

# Visualize mesh (elements wireframe)
fea_vis = fea.ChVisualizationFEAmesh(mesh)
fea_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)  # no scalar data plot
fea_vis.SetWireframe(True)
fea_vis.SetDrawNodes(False)
mesh.AddVisualization(fea_vis)

application.AssetBindAll()
application.AssetUpdateAll()

# -----------------------------------------------------------------------------
# 7. Simulation loop
# -----------------------------------------------------------------------------
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()