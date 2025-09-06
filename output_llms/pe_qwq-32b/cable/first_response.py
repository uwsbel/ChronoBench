import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    # Initialize Chrono system with SMC solver for deformable bodies
    sys = chrono.ChSystemSMC()
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity vector

    # Setup Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('ANCF Beam Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chronoirr.ChVectorD(0, 3, 10))
    vis.AddTypicalLights()
    sys.SetVisualSystem(vis)

    # Create FEA mesh and add to system
    mesh = fea.ChMesh()
    sys.Add(mesh)

    # Beam parameters
    length = 5.0          # Total length of the beam
    num_elements = 10     # Number of ANCF cable elements
    nodes = []

    # Create nodes along the beam's initial position (along x-axis)
    for i in range(num_elements + 1):
        x = i * length / num_elements
        node = fea.ChNodeFEAxyzrot()
        node.SetPos(chrono.ChVectorD(x, 0, 0))
        mesh.AddNode(node)
        nodes.append(node)

    # Constraint: Fix first node's position (hinge support)
    link = chrono.ChLinkPointLock()
    link.Initialize(nodes[0], chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    sys.Add(link)

    # Create ANCF cable elements between consecutive nodes
    for i in range(num_elements):
        nodeA = nodes[i]
        nodeB = nodes[i+1]
        element = fea.ChElementCableANCF()
        element.SetNodes(nodeA, nodeB)
        
        # Material properties (steel-like)
        element.Set_E(2e11)    # Young's modulus (Pa)
        element.Set_Area(0.001) # Cross-sectional area (m²)
        element.Set_rho(7850)  # Density (kg/m³)
        element.Set_R(0.01)    # Radius for visualization (m)
        
        mesh.AddElement(element)

    # Add visualization for the FEA mesh
    vis_shape = fea.ChVisualShapeFEAmesh()
    vis_shape.SetFEMdataType(fea.VisualizationType.VISUALIZATION_MESH)
    vis_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))  # Gray color
    mesh.AddAsset(vis_shape)

    # Simulation loop
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(0.0005)  # Smaller time step for stability

if __name__ == '__main__':
    main()