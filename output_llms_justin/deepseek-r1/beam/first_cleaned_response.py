import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  

    
    mesh = fea.ChMesh()
    system.Add(mesh)

    
    density = 7850  
    E = 2e11        
    nu = 0.3        
    beam_width = 0.1 
    beam_height = 0.2 
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(beam_width, beam_height)
    section.SetDensity(density)
    section.SetYoungModulus(E)
    section.SetPoissonRatio(nu)

    
    length = 10.0  
    num_elements = 20
    dx = length / num_elements
    nodes = []
    for i in range(num_elements + 1):
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * dx, 0, 0)))
        node.SetMass(0)
        mesh.AddNode(node)
        nodes.append(node)

    
    constraint = fea.ChLinkPointFrame()
    constraint.Initialize(nodes[0], nodes[0])
    mesh.Add(constraint)

    
    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i + 1])
        element.SetSection(section)
        mesh.Add(element)

    
    force = chrono.ChVectorD(0, -1000, 0)  
    nodes[-1].SetForce(force)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Beam FEM Demo")
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(5, -10, 5), chrono.ChVectorD(5, 0, 0))
    vis.AddSkyBox()
    vis.AddLightDirectional()

    
    visualizer = fea.ChVisualizationFEAmesh(mesh)
    visualizer.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
    visualizer.SetColorscaleMinMax(-500, 500)
    visualizer.SetSmoothFaces(True)
    visualizer.SetWireframe(False)
    mesh.AddVisualizationFEA(visualizer)

    
    step_size = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step_size)

if __name__ == "__main__":
    main()