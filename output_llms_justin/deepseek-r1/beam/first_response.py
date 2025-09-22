import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    # 1. Initialize system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable gravity

    # 2. Create mesh container
    mesh = fea.ChMesh()
    system.Add(mesh)

    # 3. Define material properties
    density = 7850  # kg/m³ (steel)
    E = 2e11        # Young's modulus (Pa)
    nu = 0.3        # Poisson's ratio
    beam_width = 0.1 # Cross-section width (m)
    beam_height = 0.2 # Cross-section height (m)
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(beam_width, beam_height)
    section.SetDensity(density)
    section.SetYoungModulus(E)
    section.SetPoissonRatio(nu)

    # 4. Create nodes
    length = 10.0  # Beam length (m)
    num_elements = 20
    dx = length / num_elements
    nodes = []
    for i in range(num_elements + 1):
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * dx, 0, 0)))
        node.SetMass(0)
        mesh.AddNode(node)
        nodes.append(node)

    # 5. Fix first node (cantilever)
    constraint = fea.ChLinkPointFrame()
    constraint.Initialize(nodes[0], nodes[0])
    mesh.Add(constraint)

    # 6. Create beam elements
    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i + 1])
        element.SetSection(section)
        mesh.Add(element)

    # 7. Apply force at last node
    force = chrono.ChVectorD(0, -1000, 0)  # 1000 N downward
    nodes[-1].SetForce(force)

    # 8. Setup visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Beam FEM Demo")
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(5, -10, 5), chrono.ChVectorD(5, 0, 0))
    vis.AddSkyBox()
    vis.AddLightDirectional()

    # Visualize beam elements
    visualizer = fea.ChVisualizationFEAmesh(mesh)
    visualizer.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
    visualizer.SetColorscaleMinMax(-500, 500)
    visualizer.SetSmoothFaces(True)
    visualizer.SetWireframe(False)
    mesh.AddVisualizationFEA(visualizer)

    # 9. Simulation loop
    step_size = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step_size)

if __name__ == "__main__":
    main()