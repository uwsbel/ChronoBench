import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


veh = chrono_vehicle.HMMWV(system)
veh.SetChassisFixed(False)
veh.SetChassisMass(1500)
veh.SetChassisInertiaXX(chrono.ChVectorD(100, 100, 100))
veh.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
veh.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))


veh.SetTireModel(chrono_vehicle.RigidTire())


system.Add(veh.GetChassisBody())
for axle in veh.GetAxles():
    system.Add(axle)


terrain = chrono.ChTerrainSCM(system)
terrain.SetName("Terrain")
terrain.SetFile("terrain.txt")
terrain.SetScale(1)
terrain.SetHeightfieldData(chrono.ChHeightfieldData(256, 256, 1, 1, 1, 1))
terrain.SetSoilParameters(chrono.ChSoilParameters(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9))
terrain.SetMovingPatchEnabled(True)
terrain.SetMovingPatchFollowsChassis(veh.GetChassisBody())
terrain.SetVisualizationType(chrono.ChTerrainSCM.VISUALIZATION_SINKAGE_FALSE_COLOR)


system.Add(terrain)


driver = chrono_vehicle.InteractiveDriver(veh)
driver.SetSteeringRange(chrono.ChReal(-1.57, 1.57))
driver.SetThrottleRange(chrono.ChReal(0, 1))
driver.SetBrakingRange(chrono.ChReal(0, 1))


vis = chronoirr.ChVisualSystemIrrlicht(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.SetCameraRig(chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


for part in veh.GetParts():
    vis.AddVisualizationMesh(part, chrono.ChVisualizationType.MESH_WIREFRAME)


while True:
    system.DoStepDynamics(0.02)
    vis.Render()
    chrono.ChUtils::Sleep(0.02)