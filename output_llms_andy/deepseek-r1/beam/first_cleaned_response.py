import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))

    
    ground = chrono.ChBody()
    ground.SetFixed(True)
    system.Add(ground)

    
    mesh = fea.ChMesh()
    material = fea.ChBeamSectionEulerAdvanced()
    
    
    material.SetDensity(7800)       
    material.SetYoungModulus(210e9) 
    material.SetShearModulus(80e9)  
    material.SetRayleighDamping(0.05)

    
    section = fea.ChBeamSectionCircular()
    section.SetDiameter(0.02)  
    material.SetSection(section)

    
    length = 4.0       
    num_elements = 10  
    node_spacing = length / num_elements

    
    nodes = []
    for i in range(num_elements + 1):
        position = chrono.ChVector3d(i * node_spacing, 0, 0)
        node = fea.ChNodeFEAxyzrot(chrono.ChFramed(position))
        node.SetMass(0)
        nodes.append(node)
        mesh.AddNode(node)

    
    constraint = fea.ChLinkNodeFrame()
    constraint.Initialize(nodes[0], ground)
    mesh.Add(constraint)

    
    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i + 1])
        element.SetSection(material)
        mesh.AddElement(element)

    
    force = chrono.ChForce()
    force.SetFz(50)  
    nodes[-1].AddForce(force)

    
    system.Add(mesh)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('PyChrono Beam Elements')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2, 0.5, 3))
    vis.AddTypicalLights()

    
    beam_vis = fea.ChVisualShapeFEA(mesh)
    beam_vis.SetFEMdataType(fea.VisDataType.BEAM_CFD)
    beam_vis.SetColorscaleMinMax(-0.2, 0.2)
    beam_vis.SetSmoothFaces(True)
    beam_vis.SetWireframe(False)
    mesh.AddVisualShapeFEA(beam_vis)

    
    node_vis = fea.ChVisualShapeFEA(mesh)
    node_vis.SetFEMglyphType(fea.VisGlyphType.NODE_DOT_POS)
    node_vis.SetFEMdataType(fea.VisDataType.NONE)
    node_vis.SetSymbolsThickness(0.008)
    mesh.AddVisualShapeFEA(node_vis)

    
    step_size = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step_size)

if __name__ == '__main__':
    main()