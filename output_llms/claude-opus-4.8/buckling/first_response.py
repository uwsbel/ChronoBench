import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

# -----------------------------------------------------------------------------
# 1. Initialize the PyChrono environment and core components
# -----------------------------------------------------------------------------

# Create the physical system
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# -----------------------------------------------------------------------------
# 2. Create a Custom Motor Function for the buckling load
# -----------------------------------------------------------------------------
# Custom function class that defines how the motor displacement evolves over time.
# This drives one end of the beam to induce axial compression (buckling).

class CustomMotorFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()

    def GetVal(self, x):
        # x is time. Apply a smooth ramping displacement to compress the beam.
        # Use a slow linear ramp with a small sinusoidal perturbation
        # to encourage buckling instability.
        A = 0.10            # max axial displacement (m)
        t_ramp = 2.0        # ramp duration (s)
        if x < t_ramp:
            disp = A * (x / t_ramp)
        else:
            disp = A
        # small lateral-inducing oscillation
        perturb = 0.0005 * math.sin(6.28 * x)
        return -(disp + perturb)   # negative -> push inward (compression)

    def Clone(self):
        return CustomMotorFunction()


# -----------------------------------------------------------------------------
# 3. Build the FEA Mesh and Beam Elements
# -----------------------------------------------------------------------------

# Create a mesh container that will hold FEA nodes and elements
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# Define a cross-sectional / material property for the beam (Euler-Bernoulli)
beam_section = fea.ChBeamSectionEulerAdvanced()

# Geometric and material properties
beam_width  = 0.012     # m
beam_height = 0.012     # m
beam_section.SetAsRectangularSection(beam_width, beam_height)
beam_section.SetYoungModulus(210e9)        # steel (Pa)
beam_section.SetShearModulusFromPoisson(0.3)
beam_section.SetDensity(7800.0)            # kg/m^3
beam_section.SetRayleighDamping(0.0001)

# Beam geometry parameters
beam_length = 1.0       # total length (m)
num_elements = 16       # number of beam elements
num_nodes = num_elements + 1

# Create nodes along the Y axis (vertical beam)
beam_nodes = []
for i in range(num_nodes):
    y = beam_length * (i / num_elements)
    # ChNodeFEAxyzrot supports both displacement and rotation DOFs
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, y, 0)))
    mesh.AddNode(node)
    beam_nodes.append(node)

# Create Euler beam elements connecting consecutive nodes
beam_elements = []
for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(beam_nodes[i], beam_nodes[i + 1])
    element.SetSection(beam_section)
    mesh.AddElement(element)
    beam_elements.append(element)

# -----------------------------------------------------------------------------
# 4. Add Constraints between system parts
# -----------------------------------------------------------------------------

# Create a fixed "ground" body to anchor the bottom of the beam
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(ground)

# Fully constrain the bottom node to the ground (clamped support)
bottom_constraint = fea.ChLinkNodeFrame()
bottom_constraint.Initialize(beam_nodes[0], ground)
sys.Add(bottom_constraint)

# Also fix rotation at the base with a rotational constraint
bottom_rot_constraint = fea.ChLinkNodeSlopeFrame()
bottom_rot_constraint.Initialize(beam_nodes[0], ground)
sys.Add(bottom_rot_constraint)

# Create a moving body that pushes on the top of the beam
loader = chrono.ChBody()
loader.SetPos(chrono.ChVector3d(0, beam_length, 0))
loader.SetMass(1.0)
sys.Add(loader)

# Connect the top beam node to the loader body
top_constraint = fea.ChLinkNodeFrame()
top_constraint.Initialize(beam_nodes[-1], loader)
sys.Add(top_constraint)

# -----------------------------------------------------------------------------
# 5. Add a Linear Motor to drive the loader (applies the buckling displacement)
# -----------------------------------------------------------------------------

# Create a prismatic-type linear motor between ground guide and loader,
# constrained to move along the Y axis. We use a slider to act on the loader.
guide = chrono.ChBody()
guide.SetFixed(True)
guide.SetPos(chrono.ChVector3d(0, beam_length, 0))
sys.Add(guide)

motor = chrono.ChLinkMotorLinearPosition()
# Frame oriented so Z of motor frame aligns with global Y (motor pushes vertically)
motor_frame = chrono.ChFramed(
    chrono.ChVector3d(0, beam_length, 0),
    chrono.QuatFromAngleX(-chrono.CH_PI_2)
)
motor.Initialize(loader, guide, motor_frame)

# Assign the custom motor function to control displacement over time
motor.SetMotorFunction(CustomMotorFunction())
sys.Add(motor)

# -----------------------------------------------------------------------------
# 6. Add Visualization for the FEA mesh
# -----------------------------------------------------------------------------

# Visualize beam elements: color by bending moment (Mz)
vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-100.0, 100.0)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)

# Visualize the nodes as small coordinate systems
vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.004)
vis_nodes.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(vis_nodes)

# -----------------------------------------------------------------------------
# 7. Configure the Irrlicht visualization application
# -----------------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono FEA Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 0.6, 1.5),
              chrono.ChVector3d(0, 0.5, 0))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVector3d(2, 4, 2), 8.0)

# -----------------------------------------------------------------------------
# 8. Configure Solver and Timestepper for accurate dynamics
# -----------------------------------------------------------------------------

# Use a sparse direct solver (good for stiff FEA problems)
solver = chrono.ChSolverPardisoMKL()
sys.SetSolver(solver)
solver.LockSparsityPattern(True)

# Use an implicit HHT timestepper for stable integration of stiff systems
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetAlpha(-0.2)
timestepper.SetMaxiters(20)
timestepper.SetAbsTolerances(1e-6)
timestepper.SetStepControl(True)
sys.SetTimestepper(timestepper)

# -----------------------------------------------------------------------------
# 9. Run the simulation loop
# -----------------------------------------------------------------------------

time_step = 0.001

while vis.Run():
    vis.BeginScene()
    vis.Render()
    # Draw a reference grid
    chronoirr.drawGrid(vis, 0.1, 0.1, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0),
                                          chrono.QuatFromAngleX(chrono.CH_PI_2)),
                       chrono.ChColor(0.4, 0.4, 0.4), True)
    vis.EndScene()

    # Advance the dynamics
    sys.DoStepDynamics(time_step)

> solver = chrono.ChSolverMINRES()
> solver.SetMaxIterations(200)
> solver.SetTolerance(1e-10)
> sys.SetSolver(solver)
>