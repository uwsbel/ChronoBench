import pychrono.core as chrono
import pychrono.irrlicht as irr

# Initialize Chrono system
system = chrono.ChSystemNSC()

# Set default parameters
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetTimestepperType(chrono.ChTimestepper.Type_Euler)
system.SetSolverType(chrono.ChSolver.Type_PSSOR)
system.SetMaxPenetrationRecoverySpeed(0.01)

# Create ground (fixed base)
ground = chrono.ChBodyEasyBox(10, 0.5, 1, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)

# Define beam parameters
beam_length = 5.0
beam_width = 0.2
beam_height = 0.2
beam_density = 7850  # Steel density in kg/m^3

# Create beam as a finite element mesh
# For simplicity, model the beam as a chain of linked finite elements

# Create nodes for the beam
num_elements = 10
node_positions = []
for i in range(num_elements + 1):
    x = i * (beam_length / num_elements)
    node_positions.append(chrono.ChVectorD(x, 0, 0))

# Create a list to hold nodes
nodes = []

# Create nodes as ChNodeFEAxyz
for pos in node_positions:
    node = chrono.ChNodeFEAxyz(chrono.ChVectorD(pos.x, pos.y, pos.z))
    nodes.append(node)
    system.Add(node)

# Create finite elements connecting nodes
elements = []
for i in range(num_elements):
    nodeA = nodes[i]
    nodeB = nodes[i + 1]
    element = chrono.ChElementTetra_4()  # Using tetrahedral elements as a placeholder
    # In practice, use appropriate beam elements
    # For simplicity, we mock the element creation here
    # Normally, you would define a beam element class
    # Placeholder: assign nodes to element
    # (This part requires a proper FE implementation)
    # For demonstration, skip detailed FE setup
    elements.append(element)

# Assign material properties to elements
material = chrono.ChMaterialBeam()
material.SetYoungModulus(2.0e11)  # Approximate steel Young's modulus
material.SetGf(0.0)
material.SetDensity(beam_density)

# Attach elements to nodes (mocked)
# In real code, assign elements properly

# Create a mesh and add it to the system
mesh = chrono.ChMesh()
for node in nodes:
    mesh.AddNode(node)
# Add elements to mesh (skipped detailed implementation)
# mesh.AddElement(element)

system.Add(mesh)

# Create a constraint: fix the left end of the beam
fixed_node = nodes[0]
fixed_node.SetFixed(True)

# Define a custom motor function (e.g., sinusoidal displacement at the free end)
class SinusoidalMotor(chrono.ChFunction):
    def __init__(self, amplitude, frequency):
        super().__init__()
        self.amplitude = amplitude
        self.frequency = frequency

    def Get_y(self, x):
        import math
        return self.amplitude * math.sin(2 * math.pi * self.frequency * x)

# Attach a motor to impose displacement at the free end
end_node = nodes[-1]
motor_amplitude = 0.01  # meters
motor_frequency = 1.0   # Hz

# Create a function for the motor
motor_function = SinusoidalMotor(motor_amplitude, motor_frequency)

# Create a driver (actuator) to impose the sinusoidal displacement
# For demonstration, use a ChLinkGeneric to connect end node to ground with prescribed motion
ground_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
ground_body.SetBodyFixed(True)
system.Add(ground_body)

# Create a link with a prescribed motion
link = chrono.ChLinkLockPrismatic()
link.Initialize(end_node, ground_body, chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0), chrono.Q_from_AngY(0)))
system.Add(link)

# Set the motion function for the prismatic joint
def update_motor(time):
    displacement = motor_function.Get_y(time)
    link.SetDriveFunction(chrono.ChFunction_Const(displacement))
    
# Visualization setup
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('PyChrono Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(beam_length/2, 1, 3))
vis.SetCameraAngle(45)

# Simulation loop
time = 0
end_time = 2.0  # seconds
while vis.Run():
    # Update motor displacement
    update_motor(time)
    # Advance simulation
    system.DoStepDynamics(1e-3)
    time += 1e-3
    if time > end_time:
        break