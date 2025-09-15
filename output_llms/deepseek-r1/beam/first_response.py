#!/usr/bin/env python3
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    # 1. Initialize system
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    # 2. Create ground body
    ground = chrono.ChBody()
    ground.SetFixed(True)
    system.Add(ground)

    # 3. Create mesh for beams
    mesh = fea.ChMesh()
    system.Add(mesh)

    # 4. Define beam properties
    beam_length = 2.0
    n_elements = 10
    node_spacing = beam_length / n_elements
    radius = 0.05  # Beam cross-section radius

    # Material properties (steel)
    density = 7800
    E = 200e9  # Elastic modulus
    nu = 0.3   # Poisson ratio
    beam_section = fea.ChBeamSectionEulerAdvanced()
    
    # Circular cross-section
    beam_section.SetCircular(radius)
    beam_section.SetYoungModulus(E)
    beam_section.SetShearModulus(E/(2*(1+nu)))
    beam_section.SetDensity(density)
    beam_section.SetRayleighDamping(0.01)

    # 5. Create nodes and elements
    nodes = []
    for i in range(n_elements + 1):
        node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(i * node_spacing, 0, 0)))
        node.SetMass(0)
        nodes.append(node)
        mesh.AddNode(node)

        # Fix first node to ground
        if i == 0:
            constraint = fea.ChLinkNodeFrame()
            constraint.Initialize(node, ground)
            system.Add(constraint)

    # Create beam elements between nodes
    for i in range(n_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i+1])
        element.SetSection(beam_section)
        mesh.AddElement(element)

    # 6. Apply force at last node
    force = chrono.ChForce()
    force.SetFz(-500)  # Apply downward force
    nodes[-1].AddForce(force)

    # 7. Visualization settings
    # For beam elements
    visualizer = fea.ChVisualShapeFEA(mesh)
    visualizer.SetFEMdataType(fea.VisualFEDataType::ELEM_BEAM_MZ)
    visualizer.SetColorscaleMinMax(-500, 500)
    visualizer.SetSmoothFaces(True)
    visualizer.SetWireframe(False)
    mesh.AddVisualShapeFEA(visualizer)

    # For nodes
    node_vis = fea.ChVisualShapeFEA(mesh)
    node_vis.SetFEMglyphType(fea.VisualFEGlyphType::NODE_DOT_POS)
    node_vis.SetFEMdataType(fea.VisualFEDataType::NONE)
    node_vis.SetSymbolsThickness(0.006)
    mesh.AddVisualShapeFEA(node_vis)

    # 8. Create Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Beam FEM - PyChrono")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.5, -2.0, 0.5), chrono.ChVector3d(1.5, 0.0, 0.0))
    vis.AddTypicalLights()

    # 9. Simulation loop
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()