import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    
    ground = chrono.ChBody()
    ground.SetFixed(True)
    system.Add(ground)

    
    mesh = fea.ChMesh()
    system.Add(mesh)

    
    beam_length = 2.0
    n_elements = 10
    node_spacing = beam_length / n_elements
    radius = 0.05  

    
    density = 7800
    E = 200e9  
    nu = 0.3   
    beam_section = fea.ChBeamSectionEulerAdvanced()
    
    
    beam_section.SetCircular(radius)
    beam_section.SetYoungModulus(E)
    beam_section.SetShearModulus(E/(2*(1+nu)))
    beam_section.SetDensity(density)
    beam_section.SetRayleighDamping(0.01)

    
    nodes = []
    for i in range(n_elements + 1):
        node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(i * node_spacing, 0, 0)))
        node.SetMass(0)
        nodes.append(node)
        mesh.AddNode(node)

        
        if i == 0:
            constraint = fea.ChLinkNodeFrame()
            constraint.Initialize(node, ground)
            system.Add(constraint)

    
    for i in range(n_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i+1])
        element.SetSection(beam_section)
        mesh.AddElement(element)

    
    force = chrono.ChForce()
    force.SetFz(-500)  
    nodes[-1].AddForce(force)

    
    
    visualizer = fea.ChVisualShapeFEA(mesh)
    visualizer.SetFEMdataType(fea.VisualFEDataType::ELEM_BEAM_MZ)
    visualizer.SetColorscaleMinMax(-500, 500)
    visualizer.SetSmoothFaces(True)
    visualizer.SetWireframe(False)
    mesh.AddVisualShapeFEA(visualizer)

    
    node_vis = fea.ChVisualShapeFEA(mesh)
    node_vis.SetFEMglyphType(fea.VisualFEGlyphType::NODE_DOT_POS)
    node_vis.SetFEMdataType(fea.VisualFEDataType::NONE)
    node_vis.SetSymbolsThickness(0.006)
    mesh.AddVisualShapeFEA(node_vis)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Beam FEM - PyChrono")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.5, -2.0, 0.5), chrono.ChVector3d(1.5, 0.0, 0.0))
    vis.AddTypicalLights()

    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()