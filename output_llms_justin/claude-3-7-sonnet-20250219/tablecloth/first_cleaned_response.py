import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr
import numpy as np


cloth_size_x = 1.0  
cloth_size_y = 1.0  
thickness = 0.01  
density = 200.0  
E = 0.5e6  
nu = 0.3  
mesh_resolution = 20  


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = fea.ChMaterialShellKirchhoff()
material.SetDensity(density)
material.SetYoungModulus(E)
material.SetPoissonRatio(nu)


mesh = fea.ChMesh()


def create_tablecloth_mesh(size_x, size_y, resolution, thickness, material):
    
    nodes = []
    elements = []
    
    dx = size_x / resolution
    dy = size_y / resolution
    
    
    for iy in range(resolution + 1):
        for ix in range(resolution + 1):
            x = ix * dx - size_x/2
            y = 0.5  
            z = iy * dy - size_y/2
            
            node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, y, z))
            mesh.AddNode(node)
            nodes.append(node)
    
    
    for iy in range(resolution):
        for ix in range(resolution):
            
            idx1 = ix + iy * (resolution + 1)
            idx2 = ix + 1 + iy * (resolution + 1)
            idx3 = ix + 1 + (iy + 1) * (resolution + 1)
            idx4 = ix + (iy + 1) * (resolution + 1)
            
            
            element = fea.ChElementShellKirchhoff4()
            element.SetNodes(nodes[idx1], nodes[idx2], nodes[idx3], nodes[idx4])
            element.SetDimensions(dx, dy)
            element.AddLayer(thickness, 0, material)
            
            
            mesh.AddElement(element)
            elements.append(element)
    
    return nodes, elements


nodes, elements = create_tablecloth_mesh(cloth_size_x, cloth_size_y, mesh_resolution, thickness, material)



corner_nodes = [nodes[0], nodes[mesh_resolution]]
for node in corner_nodes:
    constraint = fea.ChLinkPointFrame()
    constraint.Initialize(node, system.GetChFrame())
    system.Add(constraint)


system.Add(mesh)


ground_mat = chrono.ChMaterialSurfaceSMC()
ground_body = chrono.ChBodyEasyBox(2.0, 0.1, 2.0, 1000, True, True, ground_mat)
ground_body.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground_body.SetBodyFixed(True)
system.Add(ground_body)


solver = mkl.ChSolverPardisoMKL()
system.SetSolver(solver)


integrator = chrono.ChTimestepperHHT(system)
integrator.SetAlpha(-0.2)
integrator.SetMaxiters(8)
integrator.SetAbsTolerances(1e-5)
integrator.SetMode(chrono.ChTimestepperHHT.POSITION)
integrator.SetScaling(True)
integrator.SetVerbose(true)
system.SetTimestepper(integrator)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(1.5, 1.5, 1.5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


time_step = 0.001
simulation_time = 5.0  


while vis.Run() and system.GetChTime() < simulation_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    
    if system.GetChTime() > 1.0:  
        
        fold_node = nodes[mesh_resolution * (mesh_resolution + 1)]
        fold_force = chrono.ChVectorD(0.3, -0.2, 0.3)  
        fold_node.SetForce(fold_force)
    
    system.DoStepDynamics(time_step)
    
    
    print(f"Time: {system.GetChTime():.3f}")

print("Simulation complete!")