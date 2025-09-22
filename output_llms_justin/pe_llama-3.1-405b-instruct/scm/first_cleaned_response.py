import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


print("Copyright (c) 2017 projectchrono.org")


my_hmmwv = veh.HMMWV()


terrain = veh.SCMDeformableTerrain(my_hmmwv.GetSystem())


my_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngX(chrono.CH_C_PI / 2)))


terrain.AddVehicle(my_hmmwv)


my_hmmwv.SetTireType(veh.TireModel.RIGID)


my_hmmwv.AddVisualizationAssets()


terrain.SetSoilParameters(veh.SCMSoilParameters())


terrain.EnableMovingPatch(True)


terrain.SetSinkageVisualization(True)


driver = veh.ChIrrNodeApplication(my_hmmwv)


driver.SetSteeringControl(chrono.ChFunction_Const(0.0))
driver.SetThrottleControl(chrono.ChFunction_Const(0.0))
driver.SetBrakingControl(chrono.ChFunction_Const(0.0))


vis = chronoirr.ChVisualSystemIrrlicht()


vis.AttachSystem(my_hmmwv.GetSystem())


vis.SetWindowSize(1024, 768)


vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")


vis.Initialize()


while vis.Run():
    
    my_hmmwv.GetSystem().DoStepDynamics(1e-3)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.Spin(1e-3)