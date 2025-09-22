import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl
import math
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data/')  


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


mesh = fea.ChMesh()
system.Add(mesh)


beam_material = fea.ChMaterialShellReissner()
beam_material.SetYoungModulus(2.1e11)  
beam_material.SetPoissonRatio(0.3)
beam_material.SetDensity(7850)  


beam_length = 2.0  
beam_width = 0.05  
beam_thickness = 0.01  
num_elements = 20


nodes = []
for i in range(num_elements + 1):
    x = (i / num_elements) * beam_length
    node = fea.ChNodeFEAxyzrot()
    node.SetPos(chrono.ChVectorD(x, 0, 0))
    node.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)


elements = []
for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    
    
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(beam_width, beam_thickness)
    section.SetYoungModulus(beam_material.GetYoungModulus())
    section.SetGshearModulus(beam_material.GetYoungModulus() / (2 * (1 + beam_material.GetPoissonRatio())))
    section.SetDensity(beam_material.GetDensity())
    
    element.SetSection(section)
    mesh.AddElement(element)
    elements.append(element)


constraint_fixed = fea.ChNodeFEAxyzrot()
constraint_fixed.SetFixed(True)
nodes[0].SetFixed(True)


compression_body = chrono.ChBodyEasySphere(0.1, 1000, True, True)
compression_body.SetPos(chrono.ChVectorD(beam_length + 0.1, 0, 0))
compression_body.SetBodyFixed(False)
system.Add(compression_body)


constraint_compression = chrono.ChLinkMateGeneric()
constraint_compression.Initialize(nodes[-1], compression_body, False, 
                                 chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0)),
                                 chrono.ChFrameD(chrono.ChVectorD(-0.1, 0, 0)))
system.Add(constraint_compression)


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


motor = chrono.ChLinkMotorLinearForce()
motor.Initialize(compression_body, chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True))
motor_function = CompressionMotorFunction(50000, 2.0)  
motor.SetForceFunction(motor_function)
system.Add(motor)


perturbation_force = chrono.ChForce()
perturbation_force.SetF_x(chrono.ChFunction_Const(0))
perturbation_force.SetF_y(chrono.ChFunction_Sine(0, 1.0, 100))  
perturbation_force.SetF_z(chrono.ChFunction_Const(0))
nodes[num_elements // 2].AddForce(perturbation_force)


visualization = fea.ChVisualizationFEAmesh(mesh)
visualization.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_DISP_NORM)
visualization.SetColorscaleMinMax(0.0, 0.1)
visualization.SetSmoothFaces(True)
mesh.AddAsset(visualization)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling FEA Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1.0, 1.0, 2.0))
vis.AddTypicalLights()


solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)
system.SetSolver(solver)


ts = chrono.ChTimestepperEulerImplicitLinearized(system)
ts.SetMaxiters(5)
ts.SetAbsTolerances(1e-05)
system.SetTimestepper(ts)


hht_stepper = chrono.ChTimestepperHHT(system)
hht_stepper.SetAlpha(-0.2)
hht_stepper.SetMaxiters(10)
hht_stepper.SetAbsTolerances(1e-05, 1e-03)
hht_stepper.SetMode(chrono.ChTimestepperHHT.POSITION)
hht_stepper.SetScaling(True)
hht_stepper.SetVerbose(False)
system.SetTimestepper(hht_stepper)


step_size = 0.001
sim_time = 0
end_time = 5.0


displacement_data = []
force_data = []
time_data = []

print("Starting beam buckling simulation...")
print(f"Beam length: {beam_length} m")
print(f"Beam dimensions: {beam_width} x {beam_thickness} m")
print(f"Number of elements: {num_elements}")
print(f"Material: Steel (E = {beam_material.GetYoungModulus()/1e9:.1f} GPa)")


while vis.Run() and sim_time < end_time:
    vis.BeginScene()
    vis.Render()
    
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Time: {sim_time:.3f} s",
        chronoirr.recti(10, 10, 200, 30),
        chronoirr.SColor(255, 255, 255, 255)
    )
    
    
    mid_node = nodes[num_elements // 2]
    mid_displacement = mid_node.GetPos().y
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Mid-point Y displacement: {mid_displacement:.6f} m",
        chronoirr.recti(10, 40, 300, 60),
        chronoirr.SColor(255, 255, 255, 255)
    )
    
    
    time_data.append(sim_time)
    displacement_data.append(mid_displacement)
    force_data.append(motor_function.Get_y(sim_time))
    
    vis.EndScene()
    
    
    system.DoStepDynamics(step_size)
    sim_time += step_size
    
    
    if abs(mid_displacement) > 0.01 and sim_time > 1.0:
        print(f"Buckling detected at time: {sim_time:.3f} s")
        print(f"Critical force approximately: {motor_function.Get_y(sim_time):.0f} N")
        break

print("Simulation completed!")


try:
    import matplotlib.pyplot as plt
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    
    ax1.plot(time_data, displacement_data)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Mid-point Y Displacement (m)')
    ax1.set_title('Beam Mid-point Displacement vs Time')
    ax1.grid(True)
    
    
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
    
    with open('beam_buckling_data.txt', 'w') as f:
        f.write('Time(s)\tDisplacement(m)\tForce(N)\n')
        for t, d, force in zip(time_data, displacement_data, force_data):
            f.write(f'{t:.6f}\t{d:.6f}\t{force:.2f}\n')
    print("Data saved to 'beam_buckling_data.txt'")