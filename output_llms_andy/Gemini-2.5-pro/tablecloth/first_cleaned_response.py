import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import pychrono.pardisomkl as mkl 
import math
import os




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))




my_system = chrono.ChSystemSMC()
my_system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))




my_mesh = fea.ChMesh()
my_system.Add(my_mesh)





density = 150  
young_modulus = 5e5  
poisson_ratio = 0.3
thickness = 0.002  
alpha_damping = 0.05 
beta_damping = 0.001 



material = fea.ChMaterialShellKirchhoff(density, young_modulus, poisson_ratio)

material.SetRayleighDampingAlpha(alpha_damping)
material.SetRayleighDampingBeta(beta_damping)





num_nodes_x = 15  
num_nodes_z = 15  
size_x = 1.0  
size_z = 1.0  
initial_height = 0.6 


nodes_grid = [[None for _ in range(num_nodes_z)] for _ in range(num_nodes_x)]


print(f"Adding {num_nodes_x * num_nodes_z} nodes...")
for i in range(num_nodes_x):
    for j in range(num_nodes_z):
        
        x = (i / (num_nodes_x - 1) - 0.5) * size_x
        y = initial_height
        z = (j / (num_nodes_z - 1) - 0.5) * size_z
        
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))
        node.SetMass(0) 
        my_mesh.AddNode(node)
        nodes_grid[i][j] = node


print(f"Adding {(num_nodes_x - 1) * (num_nodes_z - 1)} elements...")
for i in range(num_nodes_x - 1):
    for j in range(num_nodes_z - 1):
        
        node0 = nodes_grid[i][j]
        node1 = nodes_grid[i+1][j]
        node2 = nodes_grid[i+1][j+1]
        node3 = nodes_grid[i][j+1]
        
        element = fea.ChElementShellReissner4(thickness) 
        element.SetNodes(node0, node1, node2, node3)
        
        
        
        
        element.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)
        
        my_mesh.AddElement(element)





print("Applying boundary conditions (fixing one edge)...")
for j in range(num_nodes_z):
    nodes_grid[0][j].SetFixed(True)










mkl_solver = mkl.ChSolverPardisoMKL()
my_system.SetSolver(mkl_solver)
mkl_solver.LockSparsityPattern(False) 
mkl_solver.SetVerbose(False)


my_system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
hht_stepper = my_system.GetTimestepper().StaticCast(chrono.ChTimestepperHHT)
if hht_stepper:
    hht_stepper.SetAlpha(-0.2)  
    hht_stepper.SetMaxiters(8)
    hht_stepper.SetAbsoler(1e-4)
    hht_stepper.SetReloler(1e-3)
    hht_stepper.SetMode(chrono.ChTimestepperHHT.POSITION) 
    hht_stepper.SetStepControl(False) 
    hht_stepper.SetVerbose(False)
else:
    print("Warning: Could not cast to ChTimestepperHHT. Using default HHT settings.")




print("Setting up Irrlicht visualization...")
myapplication = irr.ChVisualSystemIrrlicht()
myapplication.AttachSystem(my_system)
myapplication.SetWindowSize(1024, 768)
myapplication.SetWindowTitle('PyChrono Tablecloth Folding (Shell Elements)')
myapplication.Initialize()
myapplication.AddLogo()
myapplication.AddSkyBox()
myapplication.AddCamera(chrono.ChVector3d(1.5, 1.0, 2.0)) 
myapplication.AddTypicalLights()


vis_surface = chrono.ChVisualShapeFEA(my_mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(0.0, 5.0) 
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False) 
my_mesh.AddVisualShapeFEA(vis_surface)








vis_wire = chrono.ChVisualShapeFEA(my_mesh)
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE) 
vis_wire.SetWireframe(True) 
vis_wire.SetDefaultODEColor(chrono.ChColor(0.1,0.1,0.1)) 
my_mesh.AddVisualShapeFEA(vis_wire)





timestep = 0.005  
simulation_time = 10.0 


my_system.Setup()
my_system.Update()


print(f"Starting simulation for {simulation_time} seconds with timestep {timestep}...")
while myapplication.Run():
    myapplication.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3))
    myapplication.Render()
    
    
    
    
    
    myapplication.EndScene()
    
    my_system.DoStepDynamics(timestep)
    
    
    current_time = my_system.GetChTime()
    if int(current_time * 100) % 100 == 0 : 
         print(f"Time: {current_time:.2f} s")

    if current_time > simulation_time:
        myapplication.GetDevice().closeDevice() 
        break

print("Simulation finished.")