import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

def main():
    
    system = chrono.ChSystemSMC()
    system.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
    system.SetSolverMaxIterations(100)
    system.SetGravity(chrono.ChVector3d(0, -9.81, 0))

    
    mesh = fea.ChMesh()
    system.Add(mesh)

    
    rho = 800  
    E = 2e6    
    nu = 0.3   
    material = fea.ChMaterialShellKirchhoffIsotropic(rho, E, nu)

    
    length_x = 2.0   
    length_y = 2.0   
    num_div_x = 16   
    num_div_y = 16   

    
    nodes = []
    for iy in range(num_div_y + 1):
        for ix in range(num_div_x + 1):
            x = ix * (length_x / num_div_x)
            y = 0.05  
            z = iy * (length_y / num_div_y) - length_y/2
            
            node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))
            mesh.AddNode(node)
            nodes.append(node)

    
    for iy in range(num_div_y):
        for ix in range(num_div_x):
            n0 = iy * (num_div_x + 1) + ix
            n1 = n0 + 1
            n2 = n0 + num_div_x + 2
            n3 = n0 + num_div_x + 1
            
            element = fea.ChElementShellReissner()
            element.SetNodes(
                nodes[n0],
                nodes[n1],
                nodes[n2],
                nodes[n3]
            )
            element.SetMaterial(material)
            element.SetThickness(0.002)  
            mesh.AddElement(element)

    
    for node in nodes:
        if abs(node.GetPos().z + length_y/2) < 1e-6:  
            node.SetFixed(True)

    
    vis_mesh = fea.ChVisualizationFEAmesh(mesh)
    vis_mesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
    vis_mesh.SetShellResolution(2)
    vis_mesh.SetWireframe(True)
    mesh.AddVisualShapeFEA(vis_mesh)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Tablecloth Folding Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3, 1, 3))
    vis.AddTypicalLights()

    
    step_size = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step_size)

if __name__ == "__main__":
    main()