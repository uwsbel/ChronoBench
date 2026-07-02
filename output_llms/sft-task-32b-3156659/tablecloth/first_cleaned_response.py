import pychrono.core as chrono
import pychrono.fea as fea
import numpy as np


system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


mesh = fea.ChMesh()


material = fea.ChMaterialShellKirchhoff()
material.SetYoungModulus(2e5)  
material.SetPoissonRatio(0.3)
material.SetThickness(0.01)    


nx, ny = 20, 20  
width, height = 1.0, 1.0       
node_spacing_x = width / nx
node_spacing_y = height / ny


nodes = []
for i in range(nx + 1):
    for j in range(ny + 1):
        x = i * node_spacing_x - width/2
        y = j * node_spacing_y - height/2
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, 0))
        mesh.AddNode(node)
        nodes.append(node)


elements = []
for i in range(nx):
    for j in range(ny):
        
        n0 = nodes[i + j*(nx+1)]
        n1 = nodes[i+1 + j*(nx+1)]
        n2 = nodes[i+1 + (j+1)*(nx+1)]
        n3 = nodes[i + (j+1)*(nx+1)]
        element = fea.ChElementShellKirchhoff()
        element.SetNodes(n0, n1, n3, n2)  
        element.SetMaterial(material)
        element.SetThickness(material.GetThickness())
        mesh.AddElement(element)
        elements.append(element)


for node in nodes:
    if node.GetPos().y() < -height/2 + 1e-6:
        node.SetFixed(True)


system.Add(mesh)


solver = chrono.ChSolverPardisoMKL()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
system.SetSolver(solver)


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Tablecloth Folding Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -2, 1.5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


time_step = 0.001
end_time = 5.0
step_count = 0

while vis.Run() and step_count * time_step < end_time:
    vis.BeginScene()
    vis.Render()
    system.DoStepDynamics(time_step)
    vis.EndScene()
    step_count += 1