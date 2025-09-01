import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data/')  
system = chrono.ChSystemSMC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mesh = fea.ChMesh()


length_x = 2.0  
length_z = 1.5  
num_div_x = 20  
num_div_z = 15  
thickness = 0.002  


dx = length_x / num_div_x
dz = length_z / num_div_z


material = fea.ChMaterialShellKirchhoff(
    2000,     
    2e6,      
    0.3       
)


nodes = []
for i in range(num_div_x + 1):
    row = []
    for j in range(num_div_z + 1):
        
        x = -length_x/2 + i * dx
        y = 1.0  
        z = -length_z/2 + j * dz
        
        
        node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, y, z))
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)


elements = []
for i in range(num_div_x):
    for j in range(num_div_z):
        
        node1 = nodes[i][j]
        node2 = nodes[i+1][j]
        node3 = nodes[i+1][j+1]
        node4 = nodes[i][j+1]
        
        
        element = fea.ChElementShellBST()
        element.SetNodes(node1, node2, node3, node4)
        element.SetMaterial(material)
        element.SetThickness(thickness)
        
        
        mesh.AddElement(element)
        elements.append(element)



constraint_ground = chrono.ChBody()
constraint_ground.SetBodyFixed(True)
system.Add(constraint_ground)


corner_positions = [
    (0, 0),                           
    (num_div_x, 0),                   
    (0, num_div_z),                   
    (num_div_x, num_div_z)            
]

constraints = []
for i, j in corner_positions:
    constraint = chrono.ChLinkMateGeneric()
    constraint.Initialize(nodes[i][j], constraint_ground, 
                         chrono.ChFrameD(nodes[i][j].GetPos()))
    system.Add(constraint)
    constraints.append(constraint)



center_i = num_div_x // 2
center_j = num_div_z // 2
for i in range(center_i - 2, center_i + 3):
    for j in range(center_j - 2, center_j + 3):
        if 0 <= i <= num_div_x and 0 <= j <= num_div_z:
            nodes[i][j].SetForce(chrono.ChVectorD(0, -0.1, 0))


system.Add(mesh)


mesh.SetAutomaticGravity(True)


solver = mkl.ChSolverPardisoMKL()
solver.LockSparsityPattern(True)
system.SetSolver(solver)


integrator = chrono.ChTimestepperEulerImplicitLinearized(system)
integrator.SetMaxiters(50)
integrator.SetAbsTolerances(1e-8)
integrator.SetMode(chrono.ChTimestepperEulerImplicitLinearized.POSITION)
integrator.SetVerbose(True)
system.SetTimestepper(integrator)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 2, 3), chrono.ChVectorD(0, 0.5, 0))
vis.AddTypicalLights()


vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
vis_mesh.SetColorscaleMinMax(0.0, 5.0)
vis_mesh.SetSmoothFaces(True)
vis_mesh.SetWireframe(False)
mesh.AddAsset(vis_mesh)


vis_mesh_wireframe = fea.ChVisualizationFEAmesh(mesh)
vis_mesh_wireframe.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
vis_mesh_wireframe.SetWireframe(True)
vis_mesh_wireframe.SetDefaultMeshColor(chrono.ChColor(0.2, 0.2, 0.2))
mesh.AddAsset(vis_mesh_wireframe)


ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.3, 0.3, 0.3))
system.Add(ground)


step_size = 0.001
simulation_time = 5.0
frame_count = 0

print("Starting tablecloth folding simulation...")
print(f"Mesh: {num_div_x}x{num_div_z} elements")
print(f"Total nodes: {(num_div_x+1)*(num_div_z+1)}")
print(f"Total elements: {num_div_x*num_div_z}")


while vis.Run() and system.GetChTime() < simulation_time:
    vis.BeginScene()
    vis.Render()
    
    
    if frame_count % 100 == 0:
        print(f"Time: {system.GetChTime():.3f}s")
        
    
    if system.GetChTime() > 1.0:
        
        for constraint in constraints[2:]:  
            if constraint in system.Get_bodylist():
                system.Remove(constraint)
        constraints = constraints[:2]
    
    
    if system.GetChTime() > 2.0:
        
        wind_force = chrono.ChVectorD(0.01, 0, 0.005)
        for i in range(0, num_div_x + 1, 2):
            for j in range(0, num_div_z + 1, 2):
                current_force = nodes[i][j].GetForce()
                nodes[i][j].SetForce(current_force + wind_force)
    
    vis.EndScene()
    system.DoStepDynamics(step_size)
    frame_count += 1

print("Simulation completed!")