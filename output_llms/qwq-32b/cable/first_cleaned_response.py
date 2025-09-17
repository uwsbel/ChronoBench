import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialANCF()
material.SetYoungModulus(1e6)  
material.SetPoissionsRatio(0.3)
material.SetDensity(7850)  


beam = chrono.fea.ChBeamANCF()
beam.SetNodesNumber(21)       
beam.SetTotalLength(10.0)     
beam.SetSectionWidth(0.01)    
beam.SetSectionHeight(0.01)   
beam.SetMaterial(material)


system.Add(beam)


link = chrono.ChLinkLockPointFrame()
link.Initialize(beam.GetNodes()[0], system)
system.Add(link)


visualizer = irr.ChIrrApp(system, 'ANCF Beam Simulation', irr.dimension2du(800, 600))
visualizer.AddTypicalLights()
visualizer.AddTypicalCamera(chrono.ChVectorD(0, -10, 5), chrono.ChVectorD(0, 0, 0))
visualizer.AssetSetMargin(0)


beam.SetVisualization(chrono.fea.ChVisualizationBeam())
beam.AddAsset(chrono.ChColorAsset(0.8, 0, 0))  


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().AddBox(0.5, 0.5, 0.5)  
ground.SetCollide(False)
ground.AddAsset(chrono.ChColorAsset(0.3, 0.3, 0.3))  
visualizer.Add(ground)


visualizer.SetTimestep(0.005)
visualizer.Initialize()
visualizer.SetCameraPosition(chrono.ChVectorD(0, -10, 5))
visualizer.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
visualizer.MainLoop()