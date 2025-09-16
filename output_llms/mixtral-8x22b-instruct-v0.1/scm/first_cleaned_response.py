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
terrain.SetSoilParameters(chronoterrain.ChSCMSoilParameters(1500, 100, 0.4, 5, 0.01, 0.005, 0.001))
terrain.SetPlotType(chronoterrain.ChSCMTerrain::PLOT_SINKAGE, 0.01)
terrain.Initialize(20, 20, 0.5)


terrain.SetMovingPatch(True)
terrain.SetMovingPatchRadius(1.5)
terrain.SetMovingPatchHeight(0.1)
terrain.SetMovingPatchPosition(chrono.ChVectorD(0, 0, 0))


system.Add(terrain)


driver = chronodriver.ChIrrGuiDriver()
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)
application.AddTypicalSuspensionDriver(driver, vehicle)


application.SetTimestep(0.02)


while (application.GetDevice().run()):
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

chrono.ChronoEngine_Terminate()