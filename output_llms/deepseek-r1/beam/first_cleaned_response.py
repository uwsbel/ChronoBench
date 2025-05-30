import pychrono as chrono
import pychrono.irrlicht as irrlicht
import pychrono.fea as fea
import math


chrono.SetChronoDataPath("path/to/chrono/data/")  
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


mesh = fea.ChMesh()
system.Add(mesh)


density = 7850  
E = 2e11        
nu = 0.3        
beam_material = fea.ChContinuumElastic()
beam_material.Set_E(E)
beam_material.Set_G(E/(2*(1+nu)))
beam_material.Set_density(density)


width = 0.05   
height = 0.1   
beam_section = fea.ChBeamSectionEuler()
beam_section.SetAsRectangularSection(width, height)
beam_section.SetMaterial(beam_material)


num_elements = 10
beam_length = 2.0  
node_spacing = beam_length / num_elements


nodes = []
for i in range(num_elements + 1):
    position = chrono.ChVectorD(node_spacing * i, 0, 0)
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(position))
    node.SetMass(0)
    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(beam_section)
    mesh.AddElement(element)


nodes[0].SetFixed(True)


force = chrono.ChVectorD(0, -5000, 0)  
nodes[-1].SetForce(force)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Beam FEM Demo')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(3, 2, -1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


beam_vis = fea.ChVisualShapeFEA(mesh)
beam_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
beam_vis.SetColorscaleMinMax(-1000, 1000)
beam_vis.SetSmoothFaces(True)
beam_vis.SetWireframe(False)
mesh.AddVisualShapeFEA(beam_vis)


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)

    
    tip_pos = nodes[-1].GetPos()
    print(f"Tip displacement: {tip_pos.y:.4f} m")