import pychrono as chrono
import pychrono.irrlicht as irrlicht
import pychrono.fea as fea

def main():
    # 1. Initialize system and create physical system
    system = chrono.ChSystemNSC()
    mesh = fea.ChMesh()
    system.Add(mesh)
    
    # Material properties for beams
    density = 7850  # kg/m³ (steel)
    E = 2.1e11      # Young's modulus [Pa]
    nu = 0.3        # Poisson's ratio
    beam_width = 0.1
    beam_height = 0.2
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(beam_width, beam_height)
    section.SetYoungModulus(E)
    section.SetShearModulus(E/(2*(1+nu)))
    section.SetDensity(density)
    section.SetRayleighDamping(0.01)

    # 2. Create nodes and beam elements
    num_elements = 10
    length = 5.0    # Total beam length
    node_spacing = length / num_elements
    nodes = []

    for i in range(num_elements + 1):
        pos = chrono.ChVectorD(node_spacing * i, 0, 0)
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(pos))
        node.SetMass(0)
        if i == 0:
            # Fix first node
            fixed = fea.ChLinkPointFrame()
            fixed.Initialize(node, system.GetBody(0))
            system.Add(fixed)
        mesh.AddNode(node)
        nodes.append(node)

    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i+1])
        element.SetSection(section)
        mesh.AddElement(element)

    # 3. Add visualization
    vis_shape = fea.ChVisualShapeFEA(mesh)
    vis_shape.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
    vis_shape.SetColorscaleMinMax(-100, 100)
    vis_shape.SetSmoothFaces(True)
    vis_shape.SetWireframe(True)
    mesh.AddVisualShapeFEA(vis_shape)

    # 4. Set up Irrlicht visualization
    application = irrlicht.ChIrrApp(system, "Beam FEM Demo", irrlicht.dimension2du(1280, 720))
    application.AddLogo()
    application.AddSkyBox()
    application.AddTypicalLights()
    application.AddCamera(chrono.ChVectorD(-2, 1.5, 3), chrono.ChVectorD(0, 0, 0))
    application.AssetBindAll()
    application.AssetUpdateAll()

    # 5. Configure simulation settings
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    h = 0.001
    integrator = chrono.ChTimestepperHHT(system)
    integrator.SetStepControl(True)
    integrator.SetVerbose(False)
    system.SetTimestepper(integrator)
    
    # 6. Run simulation loop
    application.SetTimestep(h)
    while application.GetDevice().run():
        application.BeginScene()
        application.DrawAll()
        application.DoStep()
        application.EndScene()

if __name__ == "__main__":
    main()