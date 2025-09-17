import PyChrono as chrono
import PyChrono.Irrlicht as chronoirr
import PyChrono.SCM as chronosc
import PyChrono.SCM.Terrain as chronoterrain
import PyChrono.SCM.Mesh as chronomesh
import PyChrono.SCM.Tire as chronotire
import PyChrono.SCM.Driver as chronodriver


chrono.Init_ChronoEngine()


system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(system, "HMMWV on SCM Terrain", chronoirr.Dim(800, 600))


vehicle = chronosc.ChHMMWV()
vehicle.SetChassisFixed(False)
vehicle.SetPos(chrono.ChVectorD(0, 0, 0.5))
vehicle.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


tire = chronotire.ChSCMTire()
tire.SetTireType(chronotire.ChSCMTire::RIGID)
vehicle.SetTire(0, tire)
vehicle.SetTire(1, tire)
vehicle.SetTire(2, tire)
vehicle.SetTire(3, tire)


vehicle.AddVisualizationAssets(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv.obj"))


terrain = chronoterrain.ChSCMTerrain()
terrain.SetSoilParameters(chronoterrain.ChSCMSoilParameters(1000, 100, 0.5, 0.1, 100, 0.01, 0.01, 0.01, 0.01))
terrain.SetPlotType(chronoterrain.ChSCMTerrain::PLOT_SINKAGE_FALSE_COLOR)
terrain.Initialize(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(100, 100, 0.5), 50, 50)
terrain.SetMovingPatch(True)
terrain.SetMovingPatchRadius(1.5)
terrain.SetMovingPatchCenter(chrono.ChVectorD(0, 0, 0))


system.Add(terrain)


driver = chronodriver.ChSCMDriver()
driver.SetVehicle(vehicle)
driver.Initialize()


application.SetTimestep(0.02)


while application.GetDevice().run():
    system.DoStepDynamics(0.02)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

chrono.ChronoEngine_Terminate()