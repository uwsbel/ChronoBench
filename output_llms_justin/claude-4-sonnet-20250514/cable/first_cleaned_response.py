import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


system = chrono.ChSystemSMC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mesh = fea.ChMesh()


beam_length = 2.0  
num_elements = 10  
element_length = beam_length / num_elements
beam_radius = 0.02  
beam_density = 7800  
beam_E = 2.1e11  
beam_area = np.pi * beam_radius**2  
beam_I = np.pi * beam_radius**4 / 4  


material = fea.ChMaterialCableANCF()
material.SetDensity(beam_density)
material.SetYoungModulus(beam_E)


nodes = []
for i in range(num_elements + 1):
    x = i * element_length
    
    node = fea.ChNodeFEAxyzD(chrono.ChVectorD(x, 0, 0),  
                             chrono.ChVectorD(1, 0, 0))   
    mesh.AddNode(node)
    nodes.append(node)


elements = []
for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(fea.ChBeamSectionCable())
    
    
    section = element.GetSection()
    section.SetArea(beam_area)
    section.SetI(beam_I)
    section.SetDensity(beam_density)
    section.SetYoungModulus(beam_E)
    
    mesh.AddElement(element)
    elements.append(element)


system.Add(mesh)


constraint = fea.ChLinkPointFrame()
constraint.Initialize(nodes[0], system.GetGround())
system.Add(constraint)


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.GetSolver().AsIterative().SetMaxIterations(100)
system.GetSolver().AsIterative().SetTolerance(1e-10)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Cable Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 2), chrono.ChVectorD(1, 0, 0))
vis.AddTypicalLights()


mesh_visual = chronoirr.ChVisualShapeFEA(mesh)
mesh_visual.SetFEMdataType(chronoirr.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
mesh_visual.SetColorscaleMinMax(0.0, 5.50)
mesh_visual.SetSmoothFaces(True)
mesh_visual.SetWireframe(False)
vis.AttachAsset(mesh_visual)


node_visual = chronoirr.ChVisualShapeFEA(mesh)
node_visual.SetFEMglyphType(chronoirr.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
node_visual.SetFEMdataType(chronoirr.ChVisualShapeFEA.DataType_NONE)
node_visual.SetSymbolsThickness(0.02)
node_visual.SetSymbolsScale(0.1)
node_visual.SetZbufferHide(False)
vis.AttachAsset(node_visual)


time_step = 0.001
simulation_time = 5.0
time = 0


print("ANCF Cable Beam Simulation")
print(f"Beam length: {beam_length} m")
print(f"Number of elements: {num_elements}")
print(f"Number of nodes: {len(nodes)}")
print(f"Material: Steel (E = {beam_E:.1e} Pa, ρ = {beam_density} kg/m³)")
print("Starting simulation...")


while vis.Run() and time < simulation_time:
    
    vis.BeginScene()
    vis.Render()
    
    
    vis.GetDevice().getGUIEnvironment().getBuiltInFont().draw(
        f"Time: {time:.2f} s",
        chronoirr.recti(10, 10, 200, 30),
        chronoirr.SColor(255, 255, 255, 255)
    )
    
    
    if int(time / 0.5) != int((time - time_step) / 0.5):
        print(f"\nTime: {time:.2f} s")
        print("Node positions:")
        for i, node in enumerate(nodes):
            pos = node.GetPos()
            print(f"  Node {i}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        
        
        tip_pos = nodes[-1].GetPos()
        tip_deflection = abs(tip_pos.y)
        print(f"Tip deflection: {tip_deflection:.4f} m")
    
    
    system.DoStepDynamics(time_step)
    time += time_step
    
    
    vis.EndScene()

print("\nSimulation completed!")


print("\nFinal nodal positions:")
for i, node in enumerate(nodes):
    pos = node.GetPos()
    print(f"  Node {i}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")

final_tip_pos = nodes[-1].GetPos()
final_tip_deflection = abs(final_tip_pos.y)
print(f"\nFinal tip deflection: {final_tip_deflection:.4f} m")



distributed_load = beam_density * beam_area * 9.81  
theoretical_deflection = (distributed_load * beam_length**4) / (8 * beam_E * beam_I)
print(f"Theoretical deflection: {theoretical_deflection:.4f} m")
print(f"Error: {abs(final_tip_deflection - theoretical_deflection)/theoretical_deflection*100:.2f}%")