import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()
chrono.ChCollisionSystem.SetDefaultImpulseThreshold(1e-3)


veh_hmmwv = veh.HMMWV(mysystem)


veh_hmmwv.SetChassisFixed(False)
veh_hmmwv.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
veh_hmmwv.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))


mysystem.Add(veh_hmmwv.GetChassisBody())
mysystem.Add(veh_hmmwv.GetPart(0))  
mysystem.Add(veh_hmmwv.GetPart(1))  
mysystem.Add(veh_hmmwv.GetPart(2))  


terrain = veh.SCMDeformableTerrain(mysystem)


terrain.SetSoilParameters(0.1, 0.2, 0.3, 0.4, 0.5)
terrain.SetMovingPatchSize(10, 10)
terrain.SetMovingPatchCenter(chrono.ChVectorD(0, 0, 0))
terrain.SetMovingPatchFollowsChassis(True)


mysystem.Add(terrain)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.SetTypicalLenght(1.0)
vis.SetCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


veh_hmmwv.AddVisualizationMesh(vis)


terrain.AddVisualization(vis, True, False, True)


driver = veh.HMMWV_Driver(veh_hmmwv)


mysystem.SetChTimeStep(0.01)
vis.SetFrameRate(50)


while vis.Run():
    mysystem.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()