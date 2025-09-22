import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


hmmwv = veh.HMMWV()
hmmwv.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChVectorD(0, 0.5, 0))
hmmwv.SetEngineType(veh.HMMWV.EngineType_L5)
hmmwv.SetDrivetrainType(veh.HMMWV.DrivetrainType_4WD)
hmmwv.SetBrakeType(veh.HMMWV.BrakeType_HYD)
hmmwv.SetSteeringType(veh.HMMWV.SteeringType_RACK_PINION)
hmmwv.SetTireType(veh.HMMWV.TireType_RIGID)
hmmwv.AddVisualization(chrono.ChVisualMaterial())
hmmwv.AddMarkers()
sys.Add(hmmwv)


terrain = chrono.ChTerrain()
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
terrain.AddPatch(chrono.ChTerrain.Patch(chrono.ChVectorD(-10, 0, -10), chrono.ChVectorD(10, 0, -10), chrono.ChVectorD(10, 0, 10), chrono.ChVectorD(-10, 0, 10), 1, 1))
terrain.AddPatch(chrono.ChTerrain.Patch(chrono.ChVectorD(-5, 0, -5), chrono.ChVectorD(5, 0, -5), chrono.ChVectorD(5, 0, 5), chrono.ChVectorD(-5, 0, 5), 1, 1))
terrain.AddPatch(chrono.ChTerrain.Patch(chrono.ChVectorD(0, 0, -5), chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(5, 0, 5), chrono.ChVectorD(-5, 0, 5), 1, 1))
terrain.AddHeightfieldPatch(chrono.ChTerrain.HeightfieldPatch(chrono.ChVectorD(-10, 0, -10), 20, 20, 1, 1))
sys.Add(terrain)


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(hmmwv)
driver.SetSteeringMode(veh.ChIrrlichtDriver.SteeringMode_REAR_WHEELS)
driver.SetTimestep(0.01)
sys.Add(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddCamera(chrono.ChVectorD(0, 5, 10))
vis.AddTypicalLights()
vis.SetWindowSize(1024, 768)
vis.SetLogo("")


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()