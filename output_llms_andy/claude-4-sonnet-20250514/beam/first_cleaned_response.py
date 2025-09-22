import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math


print("Copyright (c) 2017 projectchrono.org")
print("Chrono version: ", chrono.CHRONO_VERSION)


system = chrono.ChSystemSMC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mesh = fea.ChMesh()



beam_section = fea.ChBeamSectionEulerAdvanced()


density = 7800  
E_modulus = 2.1e11  
poisson_ratio = 0.3
shear_modulus = E_modulus / (2 * (1 + poisson_ratio))


beam_width = 0.05  
beam_height = 0.02  
area = beam_width * beam_height
Iyy = (beam_width * beam_height**3) / 12  
Izz = (beam_height * beam_width**3) / 12  
J = 0.2 * beam_width * beam_height**3    


beam_section.SetDensity(density)
beam_section.SetYoungModulus(E_modulus)
beam_section.SetGshearModulus(shear_modulus)
beam_section.SetBeamRaleyghDamping(0.01)
beam_section.SetAsRectangularSection(beam_width, beam_height)



beam_length = 2.0  
num_elements = 10
element_length = beam_length / num_elements

nodes = []
for i in range(num_elements + 1):
    x_pos = i * element_length
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(x_pos, 0, 0)))
    nodes.append(node)
    mesh.AddNode(node)


elements = []
for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(beam_section)
    elements.append(element)
    mesh.AddElement(element)



fixed_constraint = fea.ChLinkNodeFrame()
fixed_constraint.Initialize(nodes[0], system.GetBodyByMarker(0))  
mesh.AddConstraint(fixed_constraint)


nodes[0].SetFixed(True)



force_magnitude = 1000  
force_vector = chrono.ChVectorD(0, -force_magnitude, 0)
load = fea.ChLoadNodeForce(nodes[-1], force_vector)
load_container = fea.ChLoadContainer()
load_container.Add(load)
system.Add(load_container)



distributed_load = fea.ChLoadBeamWrenchDistributed(elements[5])  
distributed_load.SetForcePerUnit(chrono.ChVectorD(0, -500, 0))  
load_container.Add(distributed_load)


system.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Beam FEA Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 3), chrono.ChVectorD(1, 0, 0))
vis.AddTypicalLights()



node_vis = fea.ChVisualShapeFEA(mesh)
node_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
node_vis.SetColorscaleMinMax(0.0, 5.0)
node_vis.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(node_vis)


element_vis = fea.ChVisualShapeFEA(mesh)
element_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
element_vis.SetColorscaleMinMax(-500, 500)
element_vis.SetSmoothFaces(False)
mesh.AddVisualShapeFEA(element_vis)


for i in range(0, len(nodes), 2):  
    coord_sys = chrono.ChVisualShapeModelFile()
    coord_sys.SetFilename(chrono.GetChronoDataFile('models/coordinate_system.obj'))
    coord_sys.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
    nodes[i].AddVisualShape(coord_sys)



stepper = chrono.ChTimestepperHHT(system)
stepper.SetAlpha(-0.2)
stepper.SetMaxiters(100)
stepper.SetAbsTolerances(1e-5)
stepper.SetMode(chrono.ChTimestepperHHT.POSITION)
stepper.SetScaling(True)
system.SetTimestepper(stepper)


time_step = 0.01
simulation_time = 5.0
time = 0


output_file = open("beam_displacement.txt", "w")
output_file.write("Time\tTip_Displacement_Y\tTip_Rotation_Z\n")

print("Starting simulation...")


while vis.Run() and time < simulation_time:
    
    vis.BeginScene()
    vis.Render()
    
    
    vis.GetGUIEnvironment().addStaticText(
        f"Time: {time:.2f}s\nTip Y-displacement: {nodes[-1].GetPos().y:.4f}m\nElements: {num_elements}",
        chronoirr.rect(10, 10, 300, 80),
        False, True, None, -1, True
    )
    
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)
    time += time_step
    
    
    tip_displacement = nodes[-1].GetPos().y
    tip_rotation = nodes[-1].GetRot().Q_to_Euler123().z
    output_file.write(f"{time:.3f}\t{tip_displacement:.6f}\t{tip_rotation:.6f}\n")
    
    
    if int(time / time_step) % 50 == 0:
        print(f"Time: {time:.2f}s, Tip displacement: {tip_displacement:.4f}m")


output_file.close()
print("Simulation completed successfully!")
print("Results saved to beam_displacement.txt")


print(f"\nFinal Results:")
print(f"Final tip displacement: {nodes[-1].GetPos().y:.4f} m")
print(f"Final tip rotation: {nodes[-1].GetRot().Q_to_Euler123().z:.4f} rad")