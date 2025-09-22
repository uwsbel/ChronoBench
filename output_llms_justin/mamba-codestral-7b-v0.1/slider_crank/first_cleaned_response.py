import pychrono


system = pychrono.ChSystemSMC()


floor = pychrono.ChTruss()
system.Add(floor)

crankshaft = pychrono.ChBody()
system.Add(crankshaft)

connecting_rod = pychrono.ChBody()
system.Add(connecting_rod)

piston = pychrono.ChBody()
system.Add(piston)





visualization = pychrono.ChIrrVisualizationData()
visualization.SetWindowSize(800, 600)
visualization.SetCameraPosition(pychrono.ChVectorD(10, 10, 10))
visualization.SetCameraTarget(pychrono.ChVectorD(0, 0, 0))
visualization.SetCameraVertical(pychrono.ChVectorD(0, 0, 1))
visualization.SetRenderMode(pychrono.ChVisualizationData.eRENDER_SHADED_SMOOTH)
visualization.SetShowAxes(True)
visualization.SetShowInfo(True)
visualization.SetShowLogo(True)


system.DoStepDynamics(0.01)