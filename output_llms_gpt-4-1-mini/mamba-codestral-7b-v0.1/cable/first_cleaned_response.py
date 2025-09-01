import pychrono
import pychrono.irrlicht


system = pychrono.ChSystemSMC()


beam = pychrono.ChBeam()
beam.SetEndPoint(pychrono.ChVectorD(1, 0, 0))  
beam.SetDirection(pychrono.ChVectorD(0, 0, 1))  
beam.SetBeamRigid(True)  
system.Add(beam)


gravity = pychrono.ChVectorD(0, 0, -9.81)  
system.Set_G_acc(gravity)


irrlicht_app = pychrono.irrlicht.ChIrrApp(system, 'Beam Simulation', pychrono.irrlicht.ChIrrWayland)
irrlicht_app.SetWindowSize(800, 600)
irrlicht_app.SetWindowTitle('Beam Simulation')
irrlicht_app.SetShowInfos(True)
irrlicht_app.SetShowFPS(True)
irrlicht_app.SetShowContactForces(False)
irrlicht_app.SetShowGlobalAxes(True)
irrlicht_app.SetShowSkeleton(False)
irrlicht_app.SetCameraVertical(pychrono.ChVectorD(0, 0, 1))
irrlicht_app.SetCameraHorizontal(pychrono.ChVectorD(1, 0, 0))
irrlicht_app.SetCameraPosition(pychrono.ChVectorD(0, 0, 5))
irrlicht_app.SetCameraLookat(pychrono.ChVectorD(0, 0, 0))
irrlicht_app.AddTypicalLogo()
irrlicht_app.AddTypicalLights()
irrlicht_app.AddBeam(beam)
irrlicht_app.AssetBindAll()
irrlicht_app.AssetUpdateAll()


irrlicht_app.SetTimestep(0.01)
irrlicht_app.SetTryRealtime(True)
irrlicht_app.Run()