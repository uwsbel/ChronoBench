import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.9)
contact_material.SetDampingF(0.5)
contact_material.SetCompliance(1e-7)
contact_material.SetCoulombFriction(0.9)
contact_material.SetRestitution(0.1)


vehicle = veh.ChVehicle()


vehicle.SetChassisFixed(False)
vehicle.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
vehicle.SetChassisOrientation(chrono.Q_from_AngX(chrono.CH_C_PI_2))


sys.Add(vehicle.GetChassisBody())


terrain = veh.SCMDeformableTerrain(sys)


terrain.SetSoilParameters(200e3, 1e-3, 0.3, 0.2, 0.01, 0.01)
terrain.SetMovingPatchSize(10, 10)
terrain.SetMovingPatchSpeed(10)
terrain.SetVisualizationMode(veh.SCMDeformableTerrain.VISUALIZATION_SINKAGE)
terrain.SetVisualizationColorMap(chrono.ColorMap.COLORMAP_JET)


sys.Add(terrain.GetGroundBody())


tire_model = veh.RigidTire()


tire_model.SetTireProperties(0.5, 0.1, 0.1, 0.1, 0.1)
tire_model.SetRimProperties(0.2, 0.1, 0.1)


vehicle.AddTire(tire_model, veh.ChVehicle.TIRE_REAR_LEFT)
vehicle.AddTire(tire_model, veh.ChVehicle.TIRE_REAR_RIGHT)
vehicle.AddTire(tire_model, veh.ChVehicle.TIRE_FRONT_LEFT)
vehicle.AddTire(tire_model, veh.ChVehicle.TIRE_FRONT_RIGHT)


mesh_vis = veh.ChVehicleMesh()


mesh_vis.SetMeshProperties(chrono.ChMesh(), chrono.ChColor(1, 1, 1))


vehicle.AddMeshVisualization(mesh_vis)


driver = veh.ChInteractiveDriver()


driver.SetSteeringGain(0.1)
driver.SetThrottleGain(0.1)
driver.SetBrakingGain(0.1)


vehicle.AddDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()


vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")
vis.SetCameraPosition(chrono.ChVectorD(0, 10, -10))
vis.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))


sys.AddVisualSystem(vis)


sys.SetTimestep(0.01)
sys.SetTolerance(1e-7)


while sys.GetChTime() < 10:
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    sys.GetContactContainer().ReportAllContacts()
    chrono.ChUtilities().LogTime()
    sys.GetSystem().DoStepDynamics()