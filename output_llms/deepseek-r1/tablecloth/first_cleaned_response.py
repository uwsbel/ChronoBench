import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


table = chrono.ChBodyEasyBox(2.2, 0.1, 2.2, 1000)  
table.SetPos(chrono.ChVectorD(0, -0.05, 0))  
table.SetBodyFixed(True)  
system.Add(table)


mesh = fea.ChMesh()
system.Add(mesh)


rho = 1000    
E = 2e7       
nu = 0.3      
thickness = 0.005  
material = fea.ChMaterialShellKirchhoff(rho, E, nu)


length_x = 2.0  
length_z = 2.0
num_div_x = 15  
num_div_z = 15


dx = length_x / num_div_x
dz = length_z / num_div_z


nodes = []
for iz in range(num_div_z + 1):
    for ix in range(num_div_x + 1):
        
        x = ix * dx - length_x / 2
        z = iz * dz - length_z / 2
        y = 0.1  
        node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, y, z))
        nodes.append(node)
        mesh.AddNode(node)


for iz in range(num_div_z):
    for ix in range(num_div_x):
        
        n0 = iz * (num_div_x + 1) + ix
        n1 = iz * (num_div_x + 1) + ix + 1
        n2 = (iz + 1) * (num_div_x + 1) + ix + 1
        n3 = (iz + 1) * (num_div_x + 1) + ix

        
        element = fea.ChElementShellReissner4()
        element.SetNodes(nodes[n0], nodes[n1], nodes[n2], nodes[n3])
        element.AddLayer(thickness, 0, material)  
        mesh.AddElement(element)


vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMdataType(fea.FEADataType_NODE_SPEED_NORM)  
vis_mesh.SetColorscaleMinMax(0, 5)  
vis_mesh.SetSmoothFaces(True)
mesh.AddVisualizationFEA(vis_mesh)


contact_material = chrono.ChMaterialSurfaceSMC()
contact_surface = fea.ChContactSurfaceMesh(mesh, contact_material)
mesh.AddContactSurface(contact_surface)
contact_surface.AddFacesFromBoundary(0.001)  


solver = chrono.ChSolverPardisoMKL()
system.SetSolver(solver)
system.SetSolverMaxIterations(100)
system.SetSolverForceTolerance(1e-10)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 1, -3), chrono.ChVectorD(0, 0, 0))  
vis.AddTypicalLights()


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)