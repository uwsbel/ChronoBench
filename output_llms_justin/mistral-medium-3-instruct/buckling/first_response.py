import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  # Set the path to Chrono data directory

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Add the required physical systems and objects
# Create a FEA beam
beam_length = 1.0
beam_width = 0.1
beam_height = 0.1
num_elements = 10

# Create the beam mesh
beam_mesh = chrono.fea.ChMesh()

# Create nodes
nodes = []
for i in range(num_elements + 1):
    x = i * beam_length / num_elements
    node = chrono.fea.ChNodeFEAxyz(chrono.ChVectorD(x, 0, 0))
    beam_mesh.AddNode(node)
    nodes.append(node)

# Create elements
for i in range(num_elements):
    element = chrono.fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetBeamInertia(beam_width*beam_height**3/12, beam_height*beam_width**3/12)
    element.SetBeamArea(beam_width * beam_height)
    element.SetYoungModulus(2.1e11)  # Steel Young's modulus
    element.SetPoissonRatio(0.3)
    beam_mesh.AddElement(element)

# Fix one end of the beam
fix_constraint = chrono.fea.ChLinkPointFrame()
fix_constraint.Initialize(nodes[0], chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(fix_constraint)

# Add the beam mesh to the system
system.Add(beam_mesh)

# 3. Set necessary default parameters
# Apply a compressive force at the free end
force_magnitude = 1000.0  # N
force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE_MODE_FORCE)
force.SetF_force(chrono.ChVectorD(0, -force_magnitude, 0))
nodes[-1].SetForce(force)

# Create a custom motor function to apply time-varying load
class CustomMotorFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.time = 0.0

    def Get_y(self, time):
        self.time = time
        # Ramp up the force over time
        if time < 1.0:
            return time * force_magnitude
        else:
            return force_magnitude

# Apply the custom motor function to the force
custom_motor = CustomMotorFunction()
nodes[-1].SetForce(chrono.ChForce())
nodes[-1].GetForce().SetMode(chrono.ChForce.FORCE_MODE_FUNCTION)
nodes[-1].GetForce().SetF_force(chrono.ChVectorD(0, -1, 0))
nodes[-1].GetForce().SetF_function(custom_motor)

# 4. Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 5, 2), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)
vis.AssetBindAll()
vis.AssetUpdateAll()

# 5. Set solver and timestepper
solver = chrono.ChSolverMINRES()
system.SetSolver(solver)
system.SetMaxItersSolverSpeed(100)
system.SetMaxItersSolverStab(100)
system.SetTolForce(1e-10)

timestepper = chrono.ChTimestepperHHT()
system.SetTimestepper(timestepper)
timestepper.SetAlpha(-0.2)
timestepper.SetMaxiters(10)
timestepper.SetTol(1e-10)

# Simulation loop
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    # Print current time and displacement
    current_time = system.GetChTime()
    displacement = nodes[-1].GetPos().y
    print(f"Time: {current_time:.2f}s, Displacement: {displacement:.4f}m")

    if current_time > 2.0:  # Stop simulation after 2 seconds
        break