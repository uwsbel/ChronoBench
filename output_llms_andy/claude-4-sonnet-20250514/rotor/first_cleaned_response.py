import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data/')  


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mesh = fea.ChMesh()


beam_length = 1.0  
beam_radius = 0.01  
beam_elements = 20  


density = 7850  
E_modulus = 210e9  
poisson_ratio = 0.3
shear_modulus = E_modulus / (2 * (1 + poisson_ratio))


material = fea.ChMaterialBeamEuler()
material.SetDensity(density)
material.SetYoungModulus(E_modulus)
material.SetShearModulus(shear_modulus)
material.SetRayleighDamping(0.01, 0.01)


section = fea.ChBeamSectionEulerAdvanced()
section.SetDensity(density)
section.SetYoungModulus(E_modulus)
section.SetShearModulus(shear_modulus)
section.SetArea(np.pi * beam_radius**2)
section.SetIyy(np.pi * beam_radius**4 / 4)
section.SetIzz(np.pi * beam_radius**4 / 4)
section.SetJ(np.pi * beam_radius**4 / 2)
section.SetRayleighDamping(0.01, 0.01)


nodes = []
for i in range(beam_elements + 1):
    x = i * beam_length / beam_elements
    node = fea.ChNodeFEAxyzrot()
    node.SetPos(chrono.ChVectorD(x, 0, 0))
    node.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)


elements = []
for i in range(beam_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(section)
    mesh.AddElement(element)
    elements.append(element)


constraint_left = fea.ChNodeFEAxyzrot()
constraint_left.SetPos(nodes[0].GetPos())
constraint_left.SetRot(nodes[0].GetRot())
constraint_left.SetFixed(True)
mesh.AddNode(constraint_left)


link_left = chrono.ChLinkMateGeneric()
link_left.Initialize(nodes[0], constraint_left, False, nodes[0].Frame(), nodes[0].Frame())
system.Add(link_left)


flywheel_mass = 5.0  
flywheel_radius = 0.1  
flywheel_thickness = 0.02  


flywheel = chrono.ChBodyEasyCylinder(flywheel_radius, flywheel_thickness, flywheel_mass)
flywheel.SetPos(chrono.ChVectorD(beam_length / 2, 0, 0))
flywheel.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.VECT_Z))
flywheel.SetBodyFixed(False)


inertia_xx = 0.5 * flywheel_mass * flywheel_radius**2
inertia_yy = inertia_zz = 0.25 * flywheel_mass * flywheel_radius**2 + (1/12) * flywheel_mass * flywheel_thickness**2
flywheel.SetInertiaXX(chrono.ChVectorD(inertia_xx, inertia_yy, inertia_zz))


system.Add(flywheel)


center_node_index = beam_elements // 2
center_node = nodes[center_node_index]


link_flywheel = chrono.ChLinkMateGeneric()
link_flywheel.Initialize(flywheel, center_node, False, flywheel.GetFrame_REF_to_abs(), center_node.Frame())
system.Add(link_flywheel)


motor_body = chrono.ChBody()
motor_body.SetBodyFixed(True)
motor_body.SetPos(chrono.ChVectorD(beam_length, 0, 0))
system.Add(motor_body)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(nodes[-1], motor_body, chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0)))


motor_speed = 50.0  
speed_function = chrono.ChFunction_Const(motor_speed)
motor.SetSpeedFunction(speed_function)
system.Add(motor)


system.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Jeffcott Rotor with IGA Beam")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 1.0), chrono.ChVectorD(0.5, 0, 0))
vis.AddTypicalLights()


mesh_vis = fea.ChVisualizationFEAmesh(mesh)
mesh_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_DISP_NORM)
mesh_vis.SetColorscaleMinMax(0.0, 0.1)
mesh_vis.SetSmoothFaces(True)
mesh_vis.SetWireframe(False)
mesh.AddAsset(mesh_vis)


nodes_vis = fea.ChVisualizationFEAmesh(mesh)
nodes_vis.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
nodes_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
nodes_vis.SetSymbolsThickness(0.008)
mesh.AddAsset(nodes_vis)


system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
system.SetSolverType(chrono.ChSolver.Type_MINRES)
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
system.SetSolver(solver)


time_step = 0.001
simulation_time = 5.0
current_time = 0


output_file = open("jeffcott_rotor_results.txt", "w")
output_file.write("Time\tFlywheel_Y\tFlywheel_Z\tBeam_Center_Y\tBeam_Center_Z\n")

print("Starting Jeffcott Rotor simulation...")
print(f"Motor speed: {motor_speed} rad/s ({motor_speed * 60 / (2 * np.pi):.1f} RPM)")
print(f"Simulation time: {simulation_time} seconds")


while vis.Run() and current_time < simulation_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)
    current_time += time_step
    
    
    if int(current_time / 0.01) != int((current_time - time_step) / 0.01):
        flywheel_pos = flywheel.GetPos()
        center_node_pos = center_node.GetPos()
        
        output_file.write(f"{current_time:.4f}\t{flywheel_pos.y:.6f}\t{flywheel_pos.z:.6f}\t"
                         f"{center_node_pos.y:.6f}\t{center_node_pos.z:.6f}\n")
        
        
        if int(current_time) != int(current_time - time_step):
            print(f"Simulation time: {current_time:.1f}s / {simulation_time}s")

output_file.close()
print("Simulation completed!")
print("Results saved to 'jeffcott_rotor_results.txt'")


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)