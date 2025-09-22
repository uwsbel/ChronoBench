import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl
import math
import numpy as np

# Initialize the PyChrono environment
chrono.SetChronoDataPath('/path/to/chrono/data/')  # Update this path as needed

# Create the physical system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create mesh container for FEA elements
mesh = fea.ChMesh()
system.Add(mesh)

# Material properties for the beam
beam_material = fea.ChMaterialShellReissner()
beam_material.SetYoungModulus(2.1e11)  # Steel Young's modulus (Pa)
beam_material.SetPoissonRatio(0.3)
beam_material.SetDensity(7850)  # Steel density (kg/m³)

# Beam dimensions
beam_length = 2.0  # meters
beam_width = 0.05  # meters
beam_thickness = 0.01  # meters
num_elements = 20

# Create nodes for the beam
nodes = []
for i in range(num_elements + 1):
    x = (i / num_elements) * beam_length
    node = fea.ChNodeFEAxyzrot()
    node.SetPos(chrono.ChVectorD(x, 0, 0))
    node.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)

# Create beam elements
elements = []
for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    
    # Cross-section properties
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(beam_width, beam_thickness)
    section.SetYoungModulus(beam_material.GetYoungModulus())
    section.SetGshearModulus(beam_material.GetYoungModulus() / (2 * (1 + beam_material.GetPoissonRatio())))
    section.SetDensity(beam_material.GetDensity())
    
    element.SetSection(section)
    mesh.AddElement(element)
    elements.append(element)

# Fix the left end of the beam (clamped boundary condition)
constraint_fixed = fea.ChNodeFEAxyzrot()
constraint_fixed.SetFixed(True)
nodes[0].SetFixed(True)

# Create a body to apply compression force
compression_body = chrono.ChBodyEasySphere(0.1, 1000, True, True)
compression_body.SetPos(chrono.ChVectorD(beam_length + 0.1, 0, 0))
compression_body.SetBodyFixed(False)
system.Add(compression_body)

# Create constraint between compression body and beam end
constraint_compression = chrono.ChLinkMateGeneric()
constraint_compression.Initialize(nodes[-1], compression_body, False, 
                                 chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0)),
                                 chrono.ChFrameD(chrono.ChVectorD(-0.1, 0, 0)))
system.Add(constraint_compression)

# Custom motor function for applying compression force
class CompressionMotorFunction(chrono.ChFunction):
    def __init__(self, max_force, ramp_time):
        super().__init__()
        self.max_force = max_force
        self.ramp_time = ramp_time
    
    def Get_y(self, x):
        if x < self.ramp_time:
            return (x / self.ramp_time) * self.max_force
        else:
            return self.max_force
    
    def Get_y_dx(self, x):
        if x < self.ramp_time:
            return self.max_force / self.ramp_time
        else:
            return 0

# Create motor for compression
motor = chrono.ChLinkMotorLinearForce()
motor.Initialize(compression_body, chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True))
motor_function = CompressionMotorFunction(50000, 2.0)  # 50kN force over 2 seconds
motor.SetForceFunction(motor_function)
system.Add(motor)

# Add small perturbation to trigger buckling
perturbation_force = chrono.ChForce()
perturbation_force.SetF_x(chrono.ChFunction_Const(0))
perturbation_force.SetF_y(chrono.ChFunction_Sine(0, 1.0, 100))  # Small sine wave force
perturbation_force.SetF_z(chrono.ChFunction_Const(0))
nodes[num_elements // 2].AddForce(perturbation_force)

# Mesh visualization
visualization = fea.ChVisualizationFEAmesh(mesh)
visualization.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_DISP_NORM)
visualization.SetColorscaleMinMax(0.0, 0.1)
visualization.SetSmoothFaces(True)
mesh.AddAsset(visualization)

# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling FEA Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1.0, 1.0, 2.0))
vis.AddTypicalLights()

# Set up solver
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)
system.SetSolver(solver)

# Set up integrator
ts = chrono.ChTimestepperEulerImplicitLinearized(system)
ts.SetMaxiters(5)
ts.SetAbsTolerances(1e-05)
system.SetTimestepper(ts)

# Alternative: Use HHT timestepper for better stability
hht_stepper = chrono.ChTimestepperHHT(system)
hht_stepper.SetAlpha(-0.2)
hht_stepper.SetMaxiters(10)
hht_stepper.SetAbsTolerances(1e-05, 1e-03)
hht_stepper.SetMode(chrono.ChTimestepperHHT.POSITION)
hht_stepper.SetScaling(True)
hht_stepper.SetVerbose(False)
system.SetTimestepper(hht_stepper)

# Simulation parameters
step_size = 0.001
sim_time = 0
end_time = 5.0

# Data collection for analysis
displacement_data = []
force_data = []
time_data = []

print("Starting beam buckling simulation...")
print(f"Beam length: {beam_length} m")
print(f"Beam dimensions: {beam_width} x {beam_thickness} m")
print(f"Number of elements: {num_elements}")
print(f"Material: Steel (E = {beam_material.GetYoungModulus()/1e9:.1f} GPa)")

# Simulation loop
while vis.Run() and sim_time < end_time:
    vis.BeginScene()
    vis.Render()
    
    # Display simulation info
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Time: {sim_time:.3f} s",
        chronoirr.recti(10, 10, 200, 30),
        chronoirr.SColor(255, 255, 255, 255)
    )
    
    # Calculate mid-point displacement for buckling analysis
    mid_node = nodes[num_elements // 2]
    mid_displacement = mid_node.GetPos().y
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Mid-point Y displacement: {mid_displacement:.6f} m",
        chronoirr.recti(10, 40, 300, 60),
        chronoirr.SColor(255, 255, 255, 255)
    )
    
    # Store data for analysis
    time_data.append(sim_time)
    displacement_data.append(mid_displacement)
    force_data.append(motor_function.Get_y(sim_time))
    
    vis.EndScene()
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    sim_time += step_size
    
    # Check for buckling (significant lateral displacement)
    if abs(mid_displacement) > 0.01 and sim_time > 1.0:
        print(f"Buckling detected at time: {sim_time:.3f} s")
        print(f"Critical force approximately: {motor_function.Get_y(sim_time):.0f} N")
        break

print("Simulation completed!")

# Optional: Save results to file
try:
    import matplotlib.pyplot as plt
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Plot displacement vs time
    ax1.plot(time_data, displacement_data)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Mid-point Y Displacement (m)')
    ax1.set_title('Beam Mid-point Displacement vs Time')
    ax1.grid(True)
    
    # Plot force vs displacement
    ax2.plot(displacement_data, force_data)
    ax2.set_xlabel('Mid-point Y Displacement (m)')
    ax2.set_ylabel('Applied Force (N)')
    ax2.set_title('Force vs Displacement (Buckling Curve)')
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig('beam_buckling_results.png', dpi=300, bbox_inches='tight')
    print("Results saved to 'beam_buckling_results.png'")
    
except ImportError:
    print("Matplotlib not available. Results not plotted.")
    # Save data to text file instead
    with open('beam_buckling_data.txt', 'w') as f:
        f.write('Time(s)\tDisplacement(m)\tForce(N)\n')
        for t, d, force in zip(time_data, displacement_data, force_data):
            f.write(f'{t:.6f}\t{d:.6f}\t{force:.2f}\n')
    print("Data saved to 'beam_buckling_data.txt'")