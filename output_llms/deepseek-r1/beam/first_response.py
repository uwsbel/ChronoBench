import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    # 1. Initialize environment
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

    # 2. Create mesh container
    mesh = fea.ChMesh()
    system.Add(mesh)

    # 3. Material properties (steel)
    density = 7800  # kg/m³
    E = 2e11        # Pa (Young's modulus)
    nu = 0.3         # Poisson ratio
    beam_material = fea.ChBeamMaterialEuler()
    beam_material.SetDensity(density)
    beam_material.SetYoungModulus(E)
    beam_material.SetPoissonRatio(nu)

    # 4. Create nodes
    length = 4.0     # Beam length
    num_elements = 8
    dx = length / num_elements
    nodes = []

    for i in range(num_elements + 1):
        position = chrono.ChVectorD(dx * i, 0, 0)
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(position))
        node.SetMass(0)
        mesh.AddNode(node)
        nodes.append(node)

    # 5. Fix first node (cantilever)
    fix_node = nodes[0]
    constraint = fea.ChLinkPointFrame()
    constraint.Initialize(fix_node, chrono.ChFrameD(fix_node.GetPos()))
    mesh.Add(constraint)

    # 6. Create beam elements
    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i + 1])
        element.SetSection(beam_material)
        mesh.AddElement(element)

    # 7. Apply force at free end
    force = chrono.ChVectorD(0, -500, 0)  # 500N downward
    nodes[-1].SetForce(force)

    # 8. Visualization setup
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Beam FEM - Cantilever')
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(2, 1, 3), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    # Visualize beam elements
    beam_visual = fea.ChVisualizationFEAmesh(mesh)
    beam_visual.SetFEMdataType(fea.E_PLOT_ELEM_BEAM_MZ)
    beam_visual.SetColorscaleMinMax(-400, 400)
    beam_visual.SetSmoothFaces(True)
    beam_visual.SetWireframe(True)
    mesh.AddVisualShapeFEA(beam_visual)

    # 9. Simulation loop
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == '__main__':
    main()