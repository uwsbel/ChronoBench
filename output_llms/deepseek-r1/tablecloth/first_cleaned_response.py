import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.mkl as mkl


system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


system.SetSolverType(chrono.ChSolver.Type.PARDISO_MKL)
solver = system.GetSolver().AsMKL()
solver.SetSparsityPatternLock(True)
solver.SetVerbose(False)


mesh = chrono.ChMesh()
system.Add(mesh)


rho = 1000  
E = 1e7     
nu = 0.3    
thickness = 0.001  
material = chrono.ChMaterialShellANCF(rho, E, nu)


length_x = 1.0
length_y = 1.0
nx = 10
ny = 10

dx = length_x / nx
dy = length_y / ny
nodes = []

for j in range(ny + 1):
    for i in range(nx + 1):
        x = i * dx
        y = j * dy
        node = chrono.ChNodeFEAxyzD(chrono.ChVector3d(x, y, 0), chrono.ChVector3d(0, 0, 1))
        nodes.append(node)
        mesh.AddNode(node)


for j in range(ny):
    for i in range(nx):
        n0 = j * (nx + 1) + i
        n1 = j * (nx + 1) + i + 1
        n2 = (j + 1) * (nx + 1) + i + 1
        n3 = (j + 1) * (nx + 1) + i

        element = chrono.ChElementShellANCF_3423()
        element.SetNodes(nodes[n0], nodes[n1], nodes[n2], nodes[n3])
        element.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)
        element.SetAlphaDamp(0.02)
        mesh.AddElement(element)


for j in range(ny + 1):
    nodes[j * (nx + 1)].SetFixed(True)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Folding Tablecloth Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(1.5, 0.5, 1.5))
vis.AddTypicalLights()


mesh_vis = chrono.ChVisualShapeFEA(mesh)
mesh_vis.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
mesh_vis.SetColorscaleMinMax(0.0, 5.0)
mesh_vis.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mesh_vis)


timestep = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(timestep)