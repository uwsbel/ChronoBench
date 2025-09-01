sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


beam_section = fea.ChBeamSectionEulerAdvanced()
beam_section.SetAsRectangularSection(0.2, 0.3)  
beam_section.SetYoungModulus(0.01e9)          
beam_section.SetShearModulus(0.01e9 * 0.3)  
beam_section.SetRayleighDamping(0.000)       


node1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  
node2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(4, 0, 0)))  


mesh.AddNode(node1)
mesh.AddNode(node2)


beam_element = fea.ChElementBeamEuler()
beam_element.SetNodes(node1, node2)
beam_element.SetSection(beam_section)
mesh.AddElement(beam_element)


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam FEA Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)


vis.DeattachSystem(sys)
vis.Destroy()