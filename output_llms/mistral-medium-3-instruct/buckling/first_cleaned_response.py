import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)
system.Add(ground)


ground_vis = chrono.ChBoxShape()
ground_vis.GetBoxGeometry().SetLengths(chrono.ChVectorD(2, 0.1, 2))
ground.AddAsset(ground_vis)
ground.GetVisualModel().AddAsset(ground_vis)


beam_length = 1.0
beam_width = 0.05
beam_height = 0.05
num_elements = 10  


beam_mesh = chrono.fea.ChMesh()
system.Add(beam_mesh)


nodes = []
for i in range(num_elements + 1):
    node = chrono.fea.ChNodeFEAxyz(chrono.ChVectorD(0, 0.5 + i * beam_length/num_elements, 0))
    nodes.append(node)
    beam_mesh.AddNode(node)


for i in range(num_elements):
    element = chrono.fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])

    
    young_modulus = 2.1e11  
    beam_cross_section = chrono.ChBeamSectionEulerRectangular()
    beam_cross_section.SetAsRectangularSection(beam_width, beam_height)
    beam_cross_section.SetYoungModulus(young_modulus)
    beam_cross_section.SetDensity(7800)  
    beam_cross_section.SetPoissonRatio(0.3)

    element.SetSection(beam_cross_section)
    beam_mesh.AddElement(element)


bottom_node = nodes[0]
bottom_node.SetFixed(True)




class AxialLoadMotor(chrono.ChLoadCustom):
    def __init__(self, node, force):
        super().__init__()
        self.node = node
        self.force = force

    def ComputeForceTorque(self):
        
        self.node.SetForce(chrono.ChVectorD(0, -self.force, 0))


axial_force = 5000.0  
axial_load = AxialLoadMotor(nodes[-1], axial_force)
system.Add(axial_load)



lateral_constraint = chrono.ChLinkLockPrismatic()
lateral_constraint.Initialize(nodes[-1], ground, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
system.Add(lateral_constraint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Beam Buckling Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, -2), chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()
vis.SetSymbolScale(0.01)


system.SetSolverType(chrono.ChSolver.Type_PSOR)  
system.SetMaxItersSolverSpeed(100)
system.SetMaxItersSolverStab(100)
system.SetTolForce(1e-5)


timestepper = chrono.ChTimestepperHHT(system)
timestepper.SetAlpha(-0.2)  
timestepper.SetMaxIters(100)
timestepper.SetTolForce(1e-5)


time_step = 0.001
simulation_time = 2.0
output_fps = 30
output_step = 1.0 / output_fps


current_time = 0
frame_number = 0

while current_time < simulation_time:
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)
    current_time += time_step

    
    if current_time < 1.0:  
        axial_force = 5000.0 + 20000.0 * current_time
        axial_load.force = axial_force

    
    if current_time >= frame_number * output_step:
        print(f"Time: {current_time:.2f}s, Axial Force: {axial_force:.1f}N")
        frame_number += 1


print("\nSimulation completed!")
print(f"Final axial force: {axial_force:.1f}N")
print("Check the visualization window for buckling behavior.")