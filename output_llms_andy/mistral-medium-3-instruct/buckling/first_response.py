import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# 1. Initialize PyChrono environment and core components
chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  # Set path to Chrono data directory
sys = chrono.ChSystemNSC()  # Non-smooth system
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Create the beam using FEA elements
beam_length = 2.0  # meters
beam_width = 0.1   # meters
beam_height = 0.1  # meters
num_elements = 10  # Number of finite elements

# Create a ChMesh for the beam
beam = chrono.ChBodyEasyMesh(
    filename='',  # Empty filename means we'll create the mesh programmatically
    density=7800,  # kg/m^3 (steel)
    make_collide=True,
    make_visual=True
)

# Create the beam geometry (box shape for simplicity)
beam_shape = chrono.ChBoxShape()
beam_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(beam_length, beam_height, beam_width))
beam.AddAsset(beam_shape)

# Add the beam to the system
sys.Add(beam)

# Create FEA nodes and elements
nodes = []
elements = []

# Create nodes along the length of the beam
for i in range(num_elements + 1):
    x = -beam_length/2 + i * beam_length/num_elements
    node = chrono.ChNodeFEAxyzrot(x, 0, 0)
    nodes.append(node)
    sys.Add(node)

# Create beam elements between nodes
for i in range(num_elements):
    element = chrono.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetYoungModulus(2.1e11)  # Steel Young's modulus
    element.SetBeamInertiaY(beam_height*beam_width**3/12)
    element.SetBeamInertiaZ(beam_width*beam_height**3/12)
    element.SetBeamSection(beam_height*beam_width)
    elements.append(element)
    sys.Add(element)

# 3. Define custom motor function (applied force at one end)
class CustomMotor(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.time = 0
        self.force_magnitude = 1000  # Newtons

    def Get_y(self, time):
        self.time = time
        # Ramp up force over first second, then maintain
        if time < 1:
            return self.force_magnitude * time
        else:
            return self.force_magnitude

# Apply the motor force to the end node
motor_force = CustomMotor()
force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE)
force.SetF_y(motor_force)
nodes[-1].AddForce(force)

# 4. Add constraints (fix one end of the beam)
constraint = chrono.ChLinkLockRevolute()
constraint.Initialize(nodes[0], chrono.ChFrameD(chrono.ChVectorD(-beam_length/2, 0, 0)))
sys.Add(constraint)

# 5. Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, -5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 5.5, -2.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)

# 6. Set up solver and timestepper
sys.SetSolverType(chrono.ChSolver.Type_MINRES)
sys.SetMaxIterationSolverSpeed(100)
sys.SetMaxIterationSolverStab(100)
sys.SetTolForce(1e-10)

timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetAlpha(-0.2)
timestepper.SetMaxIters(10)
timestepper.SetTol(1e-10)
sys.SetTimestepper(timestepper)

# Simulation loop
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)
    vis.Advance(time_step)

    # Print current force and displacement for monitoring
    current_force = motor_force.Get_y(vis.GetChTime())
    current_displacement = nodes[-1].GetPos().y
    print(f"Time: {vis.GetChTime():.2f}s, Force: {current_force:.1f}N, Displacement: {current_displacement:.4f}m")