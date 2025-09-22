import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math
import os

def main():
    
    chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))

    
    
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0)) 

    
    application = irr.ChIrrApp(system, "ANCF Cable Beam Simulation", irr.dimension2du(1024, 768))
    application.AddTypicalSky()
    application.AddTypicalLights()
    application.AddTypicalCamera(irr.vector3df(0.5, 0.5, 1.5), irr.vector3df(0.5, -0.2, 0)) 
    application.SetContactsDrawMode(irr.CONTACT_NONE) 

    
    mesh = fea.ChMesh()
    system.Add(mesh)

    
    
    beam_length = 1.0
    num_elements = 10
    num_nodes = num_elements + 1
    element_length = beam_length / num_elements
    beam_radius = 0.02 

    
    density = 7850  
    E_modulus = 210e9  
    poisson_ratio = 0.3
    alpha_damping = 0.005 
    beta_damping = 0.001  

    
    ancf_material = fea.ChMaterialBeamANCF()
    ancf_material.Set_rho(density)
    ancf_material.Set_E(E_modulus)
    ancf_material.Set_v(poisson_ratio)
    ancf_material.Set_alpha_damping(alpha_damping) 
    

    
    
    
    beam_section = fea.ChBeamSectionCable()
    beam_section.SetDiameter(2 * beam_radius)
    beam_section.SetMaterial(ancf_material)
    beam_section.SetDrawThickness(beam_radius) 

    
    nodes = []
    for i in range(num_nodes):
        
        node_pos = chrono.ChVectorD(i * element_length, 0, 0)
        
        node_dir = chrono.ChVectorD(1, 0, 0)
        
        node = fea.ChNodeFEAxyzD(node_pos, node_dir)
        nodes.append(node)
        mesh.AddNode(node)

    
    for i in range(num_elements):
        element = fea.ChElementBeamANCF()
        element.SetNodes(nodes[i], nodes[i+1])
        element.SetSection(beam_section) 
        mesh.AddElement(element)

    
    
    
    
    nodes[0].SetFixed(True)

    
    
    vis_beam_elements = chrono.ChVisualShapeFEA(mesh)
    vis_beam_elements.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE) 
    vis_beam_elements.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NONE) 
    vis_beam_elements.SetSymbolsThickness(beam_radius * 0.5) 
    vis_beam_elements.SetColor(chrono.ChColor(0.2, 0.5, 0.8)) 
    vis_beam_elements.SetWireframe(False) 
    vis_beam_elements.SetBeamSections(True) 
    mesh.AddVisualShapeFEA(vis_beam_elements)

    
    vis_beam_nodes = chrono.ChVisualShapeFEA(mesh)
    vis_beam_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE) 
    vis_beam_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS) 
    vis_beam_nodes.SetSymbolsThickness(0.01) 
    vis_beam_nodes.SetSymbolsColor(chrono.ChColor(1, 0, 0)) 
    mesh.AddVisualShapeFEA(vis_beam_nodes)
    
    
    hht_stepper = chrono.ChTimestepperHHT(mesh) 
    hht_stepper.SetAlpha(-0.2)  
    hht_stepper.SetMaxiters(100)
    hht_stepper.SetAbsolerance(1e-5)
    hht_stepper.SetMode(chrono.ChTimestepperHHT.POSITION) 
    hht_stepper.SetVerbose(False)
    hht_stepper.SetStepControl(True) 
    hht_stepper.SetModifiedNewton(False) 
    system.SetTimestepper(hht_stepper)

    
    try:
        mkl_solver = chrono.ChSolverMKL()
        system.SetSolver(mkl_solver)
        print("Using MKL solver.")
    except AttributeError:
        print("MKL solver not available, using SPARSE_LU.")
        
        
        
        
        try:
            lu_solver = chrono.ChSolverSparseLU()
            system.SetSolver(lu_solver)
        except AttributeError:
            print("SPARSE_LU solver not available, using MINRES.")
            minres_solver = chrono.ChSolverMINRES()
            system.SetSolver(minres_solver)


    
    application.SetTimestep(0.01) 
    application.SetVideoframeSaveInterval(int(1 / (application.GetTimestep() * 25))) 

    
    output_file_path = "beam_node_positions.txt"
    with open(output_file_path, "w") as outfile:
        outfile.write("Time NodeIndex X Y Z\n")

    simulation_time = 5.0 
    time = 0

    while application.GetDevice().run():
        application.BeginScene()
        application.DrawAll()
        
        
        if len(nodes) > 0:
            tip_node = nodes[-1] 
            
            
            
            
            
            
            

        application.DoStep()
        application.EndScene()
        
        time = system.GetChTime()
        if time > simulation_time:
            break
            
    print(f"Simulation finished. Node positions saved to {output_file_path}")
    application.GetDevice().closeDevice()

if __name__ == "__main__":
    main()