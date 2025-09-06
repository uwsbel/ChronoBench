import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np




cloth_size = 2.0          
cloth_thickness = 0.005   
grid_resolution = 15      
young_modulus = 5e7       
poisson_ratio = 0.3       
density = 1000            
gravity = -9.81           
simulation_time = 5.0     





system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVector3d(0, 0, gravity))


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)




material = fea.ChMaterialShellKirchhoff()
material.SetYoungModulus(young_modulus)
material.SetPoissonRatio(poisson_ratio)
material.SetDensity(density)




mesh = fea.ChMesh()
system.Add(mesh)


nodes = []
dx = cloth_size / (grid_resolution - 1)
for iy in range(grid_resolution):
    for ix in range(grid_resolution):
        x = ix * dx - cloth_size/2
        y = iy * dx - cloth_size/2
        z = 0.1  
        
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))
        node.SetMass(0)
        nodes.append(node)
        mesh.AddNode(node)


for iy in range(grid_resolution - 1):
    for ix in range(grid_resolution - 1):
        
        n0 = iy * grid_resolution + ix
        n1 = iy * grid_resolution + ix + 1
        n2 = (iy + 1) * grid_resolution + ix + 1
        n3 = (iy + 1) * grid_resolution + ix
        
        
        element = fea.ChElementShellKirchhoff()
        element.AddNodes(nodes[n0], nodes[n1], nodes[n2], nodes[n3])
        element.AddLayer(cloth_thickness, 0, material)
        element.SetAlphaDamp(0.02)  
        mesh.AddElement(element)


for idx in [0, grid_resolution-1, -1, -grid_resolution]:
    nodes[idx].SetFixed(True)




table = chrono.ChBodyEasyBox(cloth_size + 0.2, cloth_size + 0.2, 0.1, 1000)
table.SetPos(chrono.ChVector3d(0, 0, -0.05))
table.SetFixed(True)
system.Add(table)


table_mat = chrono.ChContactMaterialSMC()
table_mat.SetFriction(0.4)
table.GetCollisionModel().AddBox(table_mat, cloth_size/2 + 0.1, cloth_size/2 + 0.1, 0.05)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -2, 0.5), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()


visual_asset = fea.ChVisualShapeFEA(mesh)
visual_asset.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
visual_asset.SetColorscaleMinMax(0.0, 0.5)
visual_asset.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visual_asset)




if chrono.ChPardisoMKL.IsAvailable():
    solver = chrono.ChSolverPardisoMKL()
    solver.LockSparsityPattern(True)
    system.SetSolver(solver)
else:
    print("PardisoMKL not available. Using default solver.")

system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)




timestep = 0.001
frame_step = 1 / 30  

print("Starting simulation...")
current_time = 0
frame_number = 0

while vis.Run() and current_time < simulation_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(timestep)
    current_time += timestep
    frame_number += 1

print("Simulation completed.")