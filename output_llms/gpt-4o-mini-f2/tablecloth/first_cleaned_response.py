import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


thickness = 0.005
density = 1000
young_modulus = 1e6
poisson_ratio = 0.3


material = fea.ChMaterialShellKirchhoff()
material.SetDensity(density)
material.SetYoungModulus(young_modulus)
material.SetPoissonRatio(poisson_ratio)


mesh.AddMaterial(material)


nx = 20
ny = 10


for j in range(ny):
    for i in range(nx):
        
        x = i * 0.1
        y = j * 0.1
        z = 0.0

        
        node = fea.ChNodeFEAxyzrot(chrono.ChVector3d(x, y, z))

        
        mesh.AddNode(node)

        
        node.SetRot(node)


for j in range(ny - 1):
    for i in range(nx - 1):
        
        element = fea.ChElementShellKirchhoff()

        
        nodeA = mesh.GetNode(j * nx + i)
        nodeB = mesh.GetNode(j * nx + (i + 1))
        nodeC = mesh.GetNode((j + 1) * nx + i)

        
        element.AddNode(nodeA)
        element.AddNode(nodeB)
        element.AddNode(nodeC)

        
        element.SetMaterialLayer(material, thickness, 0, 0)

        
        mesh.AddElement(element)


fixed_node = mesh.GetNode(0)


truss = chrono.ChBodyFrame()


sys.Add(truss)


truss.AddFrameChild(mesh)


truss.SetFixed(True)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth folding')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.5, 1.5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


vis.AddMesh(mesh, chrono.ChColor(0.6, 0.6, 0.6))


solver = mkl.ChSolverPardisoMKL()
solver.SetVerbose(True)
solver.SetMaxIterations(100)
sys.SetSolver(solver)


time_step = 0.001


while vis.Run():
    
    for k in range(3):
        node = mesh.GetNode((ny - 1) * nx + k)
        node.SetForce(chrono.ChVector3d(0, -1e4, 0))

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sys.DoStepDynamics(time_step)