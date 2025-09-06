import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

def main():
    
    
    
    system = chrono.ChSystemSMC()
    system.SetGravity(chrono.ChVector3d(0, -9.81, 0))
    
    
    
    
    mesh = fea.ChMesh()
    system.Add(mesh)

    
    density = 7800  
    E = 2.0e11      
    nu = 0.3        
    beam_wy = 0.05  
    beam_wz = 0.10  

    
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(beam_wy, beam_wz)
    section.SetDensity(density)
    section.SetYoungModulus(E)
    section.SetShearModulus(E / (2 * (1 + nu)))
    section.SetRayleighDamping(0.01)

    
    
    
    beam_length = 2.0  
    num_elements = 10   
    node_spacing = beam_length / num_elements
    
    nodes = []
    for i in range(num_elements + 1):
        
        position = chrono.ChVector3d(i * node_spacing, 0, 0)
        node = fea.ChNodeFEAxyzrot(chrono.ChFramed(position))
        node.SetMass(0)  
        nodes.append(node)
        mesh.AddNode(node)
        
        
        if i == 0:
            node.SetFixed(True)
    
    
    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i+1])
        element.SetSection(section)
        mesh.AddElement(element)
    
    
    
    
    
    vis_section = fea.ChVisualShapeFEA(mesh)
    vis_section.SetFEMdataType(fea.VisFEADataType.ELEM_BEAM_MZ)
    vis_section.SetColorscaleMinMax(-20, 20)
    vis_section.SetSmoothFaces(True)
    vis_section.SetWireframe(False)
    mesh.AddVisualShapeFEA(vis_section)
    
    
    node_vis = fea.ChVisualShapeFEA(mesh)
    node_vis.SetFEMglyphType(fea.VisFEAglyphType.NODE_DOT_POS)
    node_vis.SetSymbolsThickness(0.01)
    mesh.AddVisualShapeFEA(node_vis)
    
    
    
    
    
    force_magnitude = 1000  
    force = chrono.ChVector3d(0, -force_magnitude, 0)
    nodes[-1].SetForce(force)
    
    
    
    
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
    
    
    
    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)
        
        
        if system.GetChTime() % 0.1 < time_step:
            tip_pos = nodes[-1].GetPos()
            print(f"Time: {system.GetChTime():.2f}s | Tip Displacement: {tip_pos.y:.4f}m")

if __name__ == "__main__":
    main()