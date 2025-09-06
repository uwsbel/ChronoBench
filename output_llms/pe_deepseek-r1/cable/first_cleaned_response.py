import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

def main():
    
    
    
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))  
    
    
    
    
    beam_length = 2.0        
    num_elements = 8         
    num_nodes = num_elements + 1
    node_spacing = beam_length / num_elements
    
    
    density = 1000           
    E = 1e9                  
    area = 0.1 * 0.1         
    I = (0.1**4) / 12        
    
    
    
    
    mesh = fea.ChMesh()
    system.Add(mesh)
    
    
    material = fea.ChBeamMaterialANCF(density, E, E, area, I, I, 0.3)
    
    
    
    
    nodes = []
    for i in range(num_nodes):
        
        position = chrono.ChVector3d(i * node_spacing, 0, 0)
        node = fea.ChNodeFEAxyzD(position, chrono.ChVector3d(1, 0, 0))
        node.SetMass(0)
        
        
        if i == 0:
            constraint = fea.ChLinkPointFrame()
            constraint.Initialize(node, chrono.ChFrameD(position))
            system.Add(constraint)
        
        nodes.append(node)
        mesh.AddNode(node)
    
    
    
    
    for i in range(num_elements):
        element = fea.ChElementCableANCF()
        element.SetNodes(nodes[i], nodes[i+1])
        element.SetSection(material)
        mesh.AddElement(element)
    
    
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("ANCF Cable Beam Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.5, 0.5, 2))
    vis.AddTypicalLights()
    
    
    vis_beam = fea.ChVisualShapeFEA(mesh)
    vis_beam.SetFEMdataType(fea.VisFEADataType_ANCF_BEAM_BD)
    vis_beam.SetColorscaleMinMax(-0.2, 0.2)
    vis_beam.SetSmoothFaces(True)
    vis_beam.SetWireframe(False)
    mesh.AddVisualShapeFEA(vis_beam)
    
    
    
    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.RenderFEA(mesh, fea.VisFEAmesh(True, False, False, False, False, True))
        
        
        for i, node in enumerate(nodes):
            pos = node.GetPos()
            vis.GetSceneManager().getVideoDriver().draw3DLabel(
                chronoirr.vector3df(pos.x, pos.y, pos.z),
                f"N{i}: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})".encode('utf-8'),
                chronoirr.SColor(255, 255, 150, 0)
            )
        
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()