import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)


beam_length = 2.0           
beam_width_y = 0.1          
beam_width_z = 0.1          
young_modulus = 0.01e9      
shear_modulus = 0.003e9     
density = 1000              


msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_width_y, beam_width_z)
msection.SetYoungModulus(young_modulus)
msection.SetShearModulus(shear_modulus)
msection.SetDensity(density)
msection.SetRayleighDamping(0.000)  



node_start = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
node_start.SetFixed(True)
mesh.AddNode(node_start)


node_mid = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_length/2, 0, 0)))
mesh.AddNode(node_mid)


node_end = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_length, 0, 0)))
mesh.AddNode(node_end)



element1 = fea.ChElementBeamEuler()
element1.SetNodes(node_start, node_mid)
element1.SetSection(msection)
mesh.AddElement(element1)


element2 = fea.ChElementBeamEuler()
element2.SetNodes(node_mid, node_end)
element2.SetSection(msection)
mesh.AddElement(element2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Finite Element Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 1))  
vis.AddTypicalLights()


mesh_vis = chrono.ChVisualShapeFEA(mesh)
mesh_vis.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM)
mesh_vis.SetColor(chrono.ChColor(0, 0, 1))  
mesh.AddVisualShapeFEA(mesh_vis)


time_step = 0.001  
while vis.Run():
    
    sys.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()