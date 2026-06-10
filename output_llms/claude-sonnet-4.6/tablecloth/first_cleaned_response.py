import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

try:
    import pychrono.pardisomkl as mkl
    USE_MKL = True
except ImportError:
    USE_MKL = False
    print("PardisoMKL not available, falling back to default solver.")






system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))






cloth_length_x = 1.0       
cloth_length_z = 1.0       
num_elements_x = 20        
num_elements_z = 20        


num_nodes_x = num_elements_x + 1
num_nodes_z = num_elements_z + 1


dx = cloth_length_x / num_elements_x
dz = cloth_length_z / num_elements_z


thickness   = 0.002        
density     = 500.0        
E_modulus   = 1.0e5        
poisson     = 0.3          
alpha_damp  = 0.01         





mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)


material = fea.ChElasticityKirchhoffIsothropic(E_modulus, poisson)
shell_material = fea.ChMaterialShellKirchhoff(material)
shell_material.SetDensity(density)


nodes = []  

for ix in range(num_nodes_x):
    row = []
    for iz in range(num_nodes_z):
        x = ix * dx - cloth_length_x / 2.0   
        y = 0.5                                
        z = iz * dz - cloth_length_z / 2.0

        node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, y, z))
        node.SetMass(0.0)   
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)


for ix in range(num_elements_x):
    for iz in range(num_elements_z):
        
        n00 = nodes[ix    ][iz    ]
        n10 = nodes[ix + 1][iz    ]
        n11 = nodes[ix + 1][iz + 1]
        n01 = nodes[ix    ][iz + 1]

        
        elem1 = fea.ChElementShellBST()
        elem1.SetNodes(n00, n10, n11, None, None, None)
        elem1.AddLayer(thickness, 0.0, shell_material)
        elem1.SetAlphaDamp(alpha_damp)
        mesh.AddElement(elem1)

        
        elem2 = fea.ChElementShellBST()
        elem2.SetNodes(n00, n11, n01, None, None, None)
        elem2.AddLayer(thickness, 0.0, shell_material)
        elem2.SetAlphaDamp(alpha_damp)
        mesh.AddElement(elem2)






corner_nodes = [
    nodes[0][0],
    nodes[num_elements_x][0],
]

for node in corner_nodes:
    node.SetFixed(True)





system.Add(mesh)





ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.8, 0.05, 0.8)
ground.AddVisualShape(ground_shape)
system.Add(ground)





if USE_MKL:
    mkl_solver = mkl.ChSolverPardisoMKL()
    mkl_solver.LockSparsityPattern(True)
    system.SetSolver(mkl_solver)
    print("Using PardisoMKL solver.")
else:
    solver = chrono.ChSolverMINRES()
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableWarmStart(True)
    system.SetSolver(solver)
    print("Using MINRES solver.")





hht = chrono.ChTimestepperHHT(system)
hht.SetAlpha(-0.2)
hht.SetMaxiters(10)
hht.SetAbsTolerances(1e-4, 1e-2)
hht.SetMode(chrono.ChTimestepperHHT.POSITION)
hht.SetScaling(True)
system.SetTimestepper(hht)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono – Tablecloth Folding (Shell Elements)")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.0, 1.5, -2.0), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_DISP_NORM)
vis_mesh.SetColorscaleMinMax(0.0, 0.4)
vis_mesh.SetSmoothFaces(True)
vis_mesh.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_mesh)


vis_mesh_wire = fea.ChVisualizationFEAmesh(mesh)
vis_mesh_wire.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
vis_mesh_wire.SetWireframe(True)
vis_mesh_wire.SetDefaultMeshColor(chrono.ChColor(0.3, 0.3, 0.3))
mesh.AddVisualShapeFEA(vis_mesh_wire)

vis.BindAll()





timestep   = 0.002   
total_time = 5.0     
frame      = 0

print("\n--- Starting Tablecloth Folding Simulation ---")
print(f"  Nodes   : {num_nodes_x * num_nodes_z}")
print(f"  Elements: {2 * num_elements_x * num_elements_z}")
print(f"  Solver  : {'PardisoMKL' if USE_MKL else 'MINRES'}")
print(f"  Duration: {total_time} s  |  dt = {timestep} s\n")

while vis.Run() and system.GetChTime() < total_time:

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(timestep)
    frame += 1

    if frame % 50 == 0:
        t = system.GetChTime()
        print(f"  t = {t:.3f} s  |  frame {frame:5d}")

print("\n--- Simulation complete ---")