import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.scm as scm


chrono.ChSystemSMC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.Initialize()


hmmwv = veh.HMMWV_Full()
hmmwv.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.VECT_X)))
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = scm.ChSCMTerrain(chrono.ChVectorD(0, 0, 0), 100, 100, 0.1, 10, 10)
terrain.SetSoilParameters(scm.SoilParameters(1000, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01))
terrain.EnableMovingPatch(True, hmmwv.GetChassisBody())
terrain.SetSinkageVisualization(True)


chrono.GetSystem().Add(hmmwv.GetChassisBody())
chrono.GetSystem().Add(terrain)


tire_model = veh.RigidTire(hmmwv.GetVehicle().GetWheel(0))
tire_model.SetVisualizationType(veh.VisualizationType_MESH)
hmmwv.GetVehicle().ReplaceTire(0, tire_model)


driver = veh.ChIrrGuiDriver(hmmwv.GetVehicle(), vis)
driver.SetSteeringDelta(0.01)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)


step_size = 0.02
chrono.GetSystem().SetTimestepperType(chrono.ChTimestepperType_HHT)
chrono.GetSystem().SetTimestep(step_size)


while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    driver.Update()

    
    chrono.GetSystem().DoStepDynamics(step_size)

    
    terrain.UpdateMovingPatch()

    vis.EndScene()