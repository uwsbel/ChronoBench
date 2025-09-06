import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# Create simulation system with HHT timestepper
system = chrono.ChSystemSMC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
stepper = system.GetTimestepper()
stepper.SetAlpha(-0.2)  # HHT parameter for structural dynamics
stepper.SetMaxiters(50)
stepper.SetAbsTolerances(1e-5)

# Create FEA mesh
mesh = fea.ChMesh()
system.Add(mesh)

# Beam parameters
beam_length = 2.0
num_elements = 10
element_length = beam_length / num_elements

cross_section_width = 0.1
cross_section_height = 0.05

young_modulus = 2.1e11  # Steel
shear_modulus = young_modulus / (2 * (1 + 0.3))  # Poisson's ratio 0.3

# Create nodes
nodes = []
for i in range(num_elements + 1):
    x = i * element_length
    node = fea.ChNodeFEAxyzrot()
    node.SetPos(chrono.ChVectorD(x, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)

# Fix first node (cantilever)
nodes[0].Fix(True)

# Create beam elements
for i in range(num_elements):
    elem = fea.ChElementBeamEuler()
    elem.SetNodes(nodes[i], nodes[i+1])
    
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(cross_section_width, cross_section_height)
    section.SetYoungModulus(young_modulus)
    section.SetShearModulus(shear_modulus)
    section.SetRayleighDamping(0.001)  # Add damping for numerical stability
    
    elem.SetSection(section)
    mesh.AddElement(elem)

# Motor setup for compression
class CompressionMotor(chrono.ChFunction):
    def Get_y(self, my_chrono, time):
        return -0.01 * time  # 1cm/s compression rate

# Create dummy body linked to end node
dummy_body = chrono.ChBody()
dummy_body.SetPos(nodes[-1].GetPos())
system.Add(dummy_body)

link_dummy_node = chrono.ChLinkLock()
link_dummy_node.Initialize(nodes[-1], dummy_body, chrono.ChFrameD())
system.Add(link_dummy_node)

# Fixed reference body
fixed_body = chrono.ChBody()
fixed_body.SetBodyFixed(True)
fixed_body.SetPos(nodes[-1].GetPos())  # Initial position same as end node
system.Add(fixed_body)

# Create motor between dummy and fixed body
motor = chrono.ChLinkMotorLinear()
motor.Initialize(dummy_body, fixed_body, chrono.ChFrameD())
motor.SetCoordinateFunction(CompressionMotor())
system.Add(motor)

# Visualization setup
visualizor = chronoirr.ChVisualSystemIrrlicht()
visualizor.AttachSystem(system)
visualizor.SetWindowSize(1024, 768)
visualizor.SetWindowTitle('Beam Buckling Simulation')
visualizor.SetCamera(chronoirr.ChVectorD(beam_length/2, 1, beam_length))
visualizor.SetAntialiasing(True)
visualizor.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visualizor.AddSkyBox()
visualizor.AddTypicalLights()

# Add visual shapes for FEA elements
for elem in mesh.GetElements():
    vis_shape = elem.CreateVisualShape()
    mesh.AddVisualShape(vis_shape)

visualizor.Initialize()

# Simulation loop
time_step = 0.0005  # Smaller step for better accuracy
while visualizor.Run():
    system.DoStepDynamics(time_step)
    visualizor.BeginScene()
    visualizor.Render()
    visualizor.EndScene()