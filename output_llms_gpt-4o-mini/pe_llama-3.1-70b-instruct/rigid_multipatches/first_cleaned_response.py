import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


sys = chrono.ChSystemSMC()


veh_hmmwv = veh.HMMWV()
veh_hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
veh_hmmwv.SetChassisCollisionType(chrono.ChCollisionType_AABB)
veh_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))
veh_hmmwv.SetEngineType(chrono.VehicleEngineModelType::PowerFunction)
veh_hmmwv.SetDrivetrainType(chrono.VehicleDrivetrainType::REAR_WHEEL_DRIVE)
veh_hmmwv.SetMeshVisualization(True)
veh_hmmwv.Initialize(sys)


terrain = veh.RigidTerrain(sys)
patch1_mat = veh.ChTerrainPatchMaterial()
patch1_mat.SetFriction(0.9)
patch1_mat.SetRestitution(0.01)
patch1 = terrain.AddPatch(patch1_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 10, 10)
patch1.SetTexture(chrono.GetChronoDataFile("asphalt.jpg"))

patch2_mat = veh.ChTerrainPatchMaterial()
patch2_mat.SetFriction(0.7)
patch2_mat.SetRestitution(0.01)
patch2 = terrain.AddPatch(patch2_mat, chrono.ChCoordsysD(chrono.ChVectorD(10, 0, 0), chrono.QUNIT), 10, 10)
patch2.SetTexture(chrono.GetChronoDataFile("grass.jpg"))

patch3_mat = veh.ChTerrainPatchMaterial()
patch3_mat.SetFriction(0.5)
patch3_mat.SetRestitution(0.01)
patch3 = terrain.AddPatch(patch3_mat, chrono.ChCoordsysD(chrono.ChVectorD(20, 0, 0), chrono.QUNIT), 10, 10)
patch3.SetTexture(chrono.GetChronoDataFile("dirt.jpg"))

patch4_mat = veh.ChTerrainPatchMaterial()
patch4_mat.SetFriction(0.3)
patch4_mat.SetRestitution(0.01)
patch4 = terrain.AddPatch(patch4_mat, chrono.ChCoordsysD(chrono.ChVectorD(30, 0, 0), chrono.QUNIT), 10, 10)
patch4.SetTexture(chrono.GetChronoDataFile("sand.jpg"))

patch5_mat = veh.ChTerrainPatchMaterial()
patch5_mat.SetFriction(0.1)
patch5_mat.SetRestitution(0.01)
patch5 = terrain.AddPatch(patch5_mat, chrono.ChCoordsysD(chrono.ChVectorD(40, 0, 0), chrono.QUNIT), 10, 10)
patch5.SetTexture(chrono.GetChronoDataFile("snow.jpg"))


bump_mat = veh.ChTerrainPatchMaterial()
bump_mat.SetFriction(0.9)
bump_mat.SetRestitution(0.01)
bump_patch = terrain.AddPatch(bump_mat, chrono.ChCoordsysD(chrono.ChVectorD(50, 0, 0), chrono.QUNIT), 10, 10)
bump_patch.SetMesh(chrono.GetChronoDataFile("bump.obj"))


heightmap_mat = veh.ChTerrainPatchMaterial()
heightmap_mat.SetFriction(0.9)
heightmap_mat.SetRestitution(0.01)
heightmap_patch = terrain.AddPatch(heightmap_mat, chrono.ChCoordsysD(chrono.ChVectorD(60, 0, 0), chrono.QUNIT), 10, 10)
heightmap_patch.SetHeightmap(chrono.GetChronoDataFile("heightmap.png"))

terrain.Initialize()


driver = veh.ChIrrGuiDriver(veh_hmmwv)
driver.SetSteeringMode(chrono.VehicleSteering::STEERING_MODEAckermann)
driver.SetThrottleMode(chrono.VehicleThrottle::THROTTLE_MODEPedal)
driver.SetBrakingMode(chrono.VehicleBraking::BRAKING_MODEPedal)
driver.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()
vis.AttachSystem(sys)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()