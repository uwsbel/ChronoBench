import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl






sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))





mesh = fea.ChMesh()
sys.Add(mesh)






density = 100.0          
E = 0.01e9               
nu = 0.3                 
thickness = 0.001        


elasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(elasticity)
material.SetDensity(density)






nx = 12                  
ny = 12                  
L_x = 1.0                
L_y = 1.0                

dx = L_x / nx
dy = L_y / ny


y_start = 0.5


nodes = []
for iy in range(ny + 1):
    row = []
    for ix in range(nx + 1):
        
        x = ix * dx - L_x / 2
        z = iy * dy - L_y / 2
        y = y_start

        
        pos = chrono.ChVector3d(x, y, z)
        dir = chrono.ChVector3d(0, 1, 0)   

        node = fea.ChNodeFEAxyzD(pos, dir)
        node.SetMass(0.0)
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)





for iy in range(ny):
    for ix in range(nx):
        
        nodeA = nodes[iy][ix]
        nodeB = nodes[iy][ix + 1]
        nodeC = nodes[iy + 1][ix + 1]
        nodeD = nodes[iy + 1][ix]

        
        element = fea.ChElementShellBST()

        
        
        
        element.SetNodes(
            nodeA, nodeB, nodeD,    
            None, None, None        
        )

        element.AddLayer(thickness, 0.0 * chrono.CH_DEG_TO_RAD, material)

        mesh.AddElement(element)






for ix in range(nx + 1):
    nodes[0][ix].SetFixed(True)






vis_mesh = chrono.ChVisualShapeFEA(mesh)
vis_mesh.SetSmoothFaces(True)
vis_mesh.SetWireframe(False)
vis_mesh.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mesh.AddVisualShapeFEA(vis_mesh)


vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(vis_nodes)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth Folding - Kirchhoff Shell")
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 1.0, -1.5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()





solver = mkl.ChSolverPardisoMKL()
solver.LockSparsityPattern(True)
sys.SetSolver(solver)


stepper = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(stepper)





timestep = 0.001

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)