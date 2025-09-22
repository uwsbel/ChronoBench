import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math






sys = chrono.ChSystemSMC() 
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


my_mesh = fea.ChMesh()
sys.Add(my_mesh)






mat_density = 7850.0  
mat_E = 210e9        
mat_nu = 0.3         
mat_G = mat_E / (2 * (1 + mat_nu)) 
mat_specific_weight = mat_density * 9.81 


beam_section_h = 0.1  
beam_section_w = 0.05 





beam_section = fea.ChBeamSectionEulerAdvanced()
beam_section.SetDensity(mat_density)
beam_section.SetYoungModulus(mat_E)
beam_section.SetGshearModulus(mat_G) 
beam_section.SetBeamRaleyghDamping(0.000) 
beam_section.SetAsRectangularSection(beam_section_w, beam_section_h)





num_elements = 10
beam_length = 2.0  
delta_x = beam_length / num_elements

nodes = []
beam_elements = []


for i in range(num_elements + 1):
    x_pos = i * delta_x
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(x_pos, 0, 0))) 
    my_mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elements):
    nodeA = nodes[i]
    nodeB = nodes[i+1]
    
    beam_element = fea.ChElementBeamEuler()
    beam_element.SetNodes(nodeA, nodeB)
    beam_element.SetSection(beam_section)
    
    
    
    
    
    beam_element.SetRestEllipticInertiaProducts(False) 
    
    
    
    
    
    

    my_mesh.AddElement(beam_element)
    beam_elements.append(beam_element)






nodes[0].SetFixed(True)


tip_force_y = -500.0  
nodes[-1].SetForce(chrono.ChVector3d(0, tip_force_y, 0))









solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableWarmStart(True)
solver.SetVerbose(False)



stepper = chrono.ChTimestepperHHT(sys) 
sys.SetTimestepper(stepper)
stepper.SetAlpha(-0.2) 
stepper.SetMaxiters(10)
stepper.SetAbsoler(1e-6)
stepper.SetMode(chrono.ChTimestepperHHT.POSITION) 
stepper.SetScaling(True)
stepper.SetStepControl(False) 
stepper.SetVerbose(False)







application = chronoirr.ChIrrApp(sys, "Beam FEM Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(beam_length * 0.5, beam_length * 0.3, -beam_length * 0.8), 
                             chronoirr.vector3df(beam_length * 0.5, 0, 0))      



vis_fea_mesh = fea.ChVisualShapeFEA(my_mesh)
vis_fea_mesh.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE) 
vis_fea_mesh.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NONE) 
vis_fea_mesh.SetSymbolsScale(0.01)
vis_fea_mesh.SetWireframe(False) 
vis_fea_mesh.SetDefault moždaBeamColor(chronoirr.SColor(255, 0, 100, 100)) 
vis_fea_mesh.SetBeamSolidShape(fea.ChVisualShapeFEA.BeamSolidShape_RECTANGULAR_SECTION) 
my_mesh.AddVisualShapeFEA(vis_fea_mesh)












application.AssetBindAll()
application.AssetUpdateAll()





application.SetTimestep(0.01)       
application.SetTryRealtime(False)   







print("Starting simulation...")

simulation_time = 0
max_simulation_time = 5.0 





while application.GetDevice().run():
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 160, 190)) 
    application.DrawAll()
    
    
    sys.DoStepDynamics(application.GetTimestep())
    
    simulation_time += application.GetTimestep()
    
    
    if int(simulation_time / application.GetTimestep()) % 10 == 0: 
        tip_node_pos = nodes[-1].GetPos()
        print(f"Time: {simulation_time:.2f} s, Tip Y-Deflection: {tip_node_pos.y:.4f} m")

    application.EndScene()

    if simulation_time > max_simulation_time:
        break
        
print("Simulation finished.")
application.GetDevice().closeDevice()