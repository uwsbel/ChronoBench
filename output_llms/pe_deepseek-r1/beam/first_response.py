import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

def main():
    # ==========================================================================
    # System Initialization
    # ==========================================================================
    system = chrono.ChSystemSMC()
    system.SetGravity(chrono.ChVector3d(0, -9.81, 0))
    
    # ==========================================================================
    # FEA Mesh Setup
    # ==========================================================================
    mesh = fea.ChMesh()
    system.Add(mesh)

    # Material properties (steel)
    density = 7800  # kg/m³
    E = 2.0e11      # Young's modulus (Pa)
    nu = 0.3        # Poisson's ratio
    beam_wy = 0.05  # Section width in y direction (m)
    beam_wz = 0.10  # Section height in z direction (m)

    # Create beam section
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(beam_wy, beam_wz)
    section.SetDensity(density)
    section.SetYoungModulus(E)
    section.SetShearModulus(E / (2 * (1 + nu)))
    section.SetRayleighDamping(0.01)

    # ==========================================================================
    # Nodes and Elements
    # ==========================================================================
    beam_length = 2.0  # Total beam length (m)
    num_elements = 10   # Number of finite elements
    node_spacing = beam_length / num_elements
    
    nodes = []
    for i in range(num_elements + 1):
        # Create nodes along the x-axis
        position = chrono.ChVector3d(i * node_spacing, 0, 0)
        node = fea.ChNodeFEAxyzrot(chrono.ChFramed(position))
        node.SetMass(0)  # Consistent mass handled by element
        nodes.append(node)
        mesh.AddNode(node)
        
        # Fix first node (cantilever constraint)
        if i == 0:
            node.SetFixed(True)
    
    # Create beam elements between nodes
    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i+1])
        element.SetSection(section)
        mesh.AddElement(element)
    
    # ==========================================================================
    # Visualization Setup
    # ==========================================================================
    # Create visualization assets
    vis_section = fea.ChVisualShapeFEA(mesh)
    vis_section.SetFEMdataType(fea.VisFEADataType.ELEM_BEAM_MZ)
    vis_section.SetColorscaleMinMax(-20, 20)
    vis_section.SetSmoothFaces(True)
    vis_section.SetWireframe(False)
    mesh.AddVisualShapeFEA(vis_section)
    
    # Add node visualization
    node_vis = fea.ChVisualShapeFEA(mesh)
    node_vis.SetFEMglyphType(fea.VisFEAglyphType.NODE_DOT_POS)
    node_vis.SetSymbolsThickness(0.01)
    mesh.AddVisualShapeFEA(node_vis)
    
    # ==========================================================================
    # Load Application
    # ==========================================================================
    # Apply vertical force at free end
    force_magnitude = 1000  # Newtons
    force = chrono.ChVector3d(0, -force_magnitude, 0)
    nodes[-1].SetForce(force)
    
    # ==========================================================================
    # Irrlicht Visualization System
    # ==========================================================================
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Beam FEA Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.5, 0.5, 2.0), chrono.ChVector3d(1.0, 0, 0))
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 12, 12, 
                chrono.ChCoordsysd(chrono.ChVector3d(0, -0.1, 0), 
                chrono.QuatFromAngleX(chrono.CH_PI_2)))
    
    # ==========================================================================
    # Simulation Loop
    # ==========================================================================
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)
        
        # Print tip displacement every 100 steps
        if system.GetChTime() % 0.1 < time_step:
            tip_pos = nodes[-1].GetPos()
            print(f"Time: {system.GetChTime():.2f}s | Tip Displacement: {tip_pos.y:.4f}m")

if __name__ == "__main__":
    main()