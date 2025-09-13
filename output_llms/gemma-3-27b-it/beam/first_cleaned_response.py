import pychrono as chrono
import pychrono.irrlicht as chronoirr






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


beam_mesh = chrono.ChBeamMesh()


beam_mesh.AddNode(chrono.ChVectorD(0, 0, 0))  
beam_mesh.AddNode(chrono.ChVectorD(1, 0, 0))  
beam_mesh.AddNode(chrono.ChVectorD(2, 0, 0))  
beam_mesh.AddNode(chrono.ChVectorD(0, 1, 0))  
beam_mesh.AddNode(chrono.ChVectorD(1, 1, 0))  
beam_mesh.AddNode(chrono.ChVectorD(2, 1, 0))  


beam_mesh.AddBeam(0, 1, 0.1, chrono.ChBeamMesh.SECTION_RECT, 0.05, 0.05)
beam_mesh.AddBeam(1, 2, 0.1, chrono.ChBeamMesh.SECTION_RECT, 0.05, 0.05)
beam_mesh.AddBeam(0, 3, 0.1, chrono.ChBeamMesh.SECTION_RECT, 0.05, 0.05)
beam_mesh.AddBeam(1, 4, 0.1, chrono.ChBeamMesh.SECTION_RECT, 0.05, 0.05)
beam_mesh.AddBeam(2, 5, 0.1, chrono.ChBeamMesh.SECTION_RECT, 0.05, 0.05)
beam_mesh.AddBeam(3, 4, 0.1, chrono.ChBeamMesh.SECTION_RECT, 0.05, 0.05)
beam_mesh.AddBeam(4, 5, 0.1, chrono.ChBeamMesh.SECTION_RECT, 0.05, 0.05)



beam_body = chrono.ChBodyEasy()
beam_body.AddAsset(beam_mesh)
beam_body.SetBodyFixed(False)
beam_body.SetMass(1)
beam_body.SetPos(chrono.ChVectorD(0,0,0))
system.Add(beam_body)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Beam Example')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 1.5, -3))
vis.AddTypicalLights()







while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)