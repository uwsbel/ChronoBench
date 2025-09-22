import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath("/path/to/data")


sys = chrono.ChSystemSMC()


veh_hmmwv = veh.HMMWV()
veh_hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
veh_hmmwv.SetChassisCollisionType(chrono.ChCollisionType_AABB)
veh_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
veh_hmmwv.SetPowertrainType(veh.PowertrainType_TORSO)
veh_hmmwv.SetDriveType(veh.DrivetrainType_REARWHEEL)
veh_hmmwv.SetRigidTireModel(True)
veh_hmmwv.SetTireVisualization(True)
veh_hmmwv.SetChassisVisualization(True)
veh_hmmwv.SetSuspensionVisualization(True)
veh_hmmwv.SetSteeringVisualization(True)
veh_hmmwv.SetWheelVisualization(True)
veh_hmmwv.SetTireForcesVisualization(True)
veh_hmmwv.SetTireForceVisualizationMode(chrono.VisualizationMode_TUBE)
veh_hmmwv.SetTireForceVisualizationRadius(0.03)
veh_hmmwv.SetTireForceVisualizationColor(chrono.ChColor(1, 0, 0))
veh_hmmwv.Initialize(sys)


terrain = veh.SCMDeformableTerrain(sys)
terrain.SetName("terrain")
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.6)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e8)
patch_mat.SetPoissonsRatio(0.3)
patch_mat.SetDampingF(0.2)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)), 10, 10)
terrain.Initialize()


terrain.SetVisualizationType(chrono.DeformableTerrainVisualizationType_HEIGHT_COLOR)
terrain.SetColorMapHeight(chrono.ColorMapHEIGHT())
terrain.SetShowNormalVectors(False)
terrain.SetShowTriangles(False)
terrain.SetShowWireframe(False)


terrain.SetMovingPatchType(chrono.DeformableTerrainMovingPatchType_OBJECT)
terrain.SetObjectToFollow(veh_hmmwv.GetChassisBody())


driver = veh.RigidDriver(veh_hmmwv.GetPart(0))
driver.SetSteeringFunction(chrono.ChFunction_Const(0))
driver.SetThrottleFunction(chrono.ChFunction_Const(0.2))
driver.SetBrakingFunction(chrono.ChFunction_Const(0))
veh_hmmwv.AddDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.AttachSystem(sys)
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()
vis.SetShowFPS(True)
vis.SetSynch(True)
vis.SetTimestep(0.01)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    veh_hmmwv.Update(0.01)
    terrain.Synchronize(0.01)
    vis.SetTimestep(0.01)