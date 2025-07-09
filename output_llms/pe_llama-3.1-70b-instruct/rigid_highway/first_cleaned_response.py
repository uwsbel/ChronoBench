import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemSMC()


chrono.SetChronoDataPath('/path/to/data')


car = veh.HMMWV()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisCollisionType(chrono.ChCollisionType_BULLET)
car.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0), chrono.QUNIT))
car.SetTireModel(chrono.TireModelType.TMEASY)
car.SetTireVisualization(True)
car.SetWheelVisualization(True)
car.SetChassisVisualization(True)
car.SetSteeringVisualization(True)
car.SetSuspensionVisualization(True)
car.Initialize(sys)


terrain = veh.RigidTerrain(car.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


terrainMesh = chrono.ChTriangleMeshConnected()
terrainMesh.LoadWavefrontMesh(chrono.GetChronoDataFile('Highway_col.obj'))
terrain.AddTriangleMesh(terrainMesh, patch_mat, chrono.ChVector3D(0, 0, 0), chrono.QUNIT)


terrainVisMesh = chrono.ChTriangleMeshConnected()
terrainVisMesh.LoadWavefrontMesh(chrono.GetChronoDataFile('Highway_vis.obj'))
terrain.AddTriangleMesh(terrainVisMesh, chrono.ChVector3D(0, 0, 0), chrono.QUNIT)


driver = veh.RSDriverHMMWV(car)
driver.SetSteeringMode(veh.SteeringMode.TYPE_ARC)
driver.SetThrottleMode(veh.ThrottleMode.TYPE_ACCELERATION)
driver.SetBrakingMode(veh.BrakingMode.TYPE_ACCELERATION)
driver.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.AttachSystem(sys)
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3D(0, 1, 6))
vis.AddCamera(chrono.ChVector3D(0, 1, 6))
vis.AddLightWithShadow(chrono.ChVector3D(1, 1, 2), chrono.ChVector3D(0, 0, 0), 10, 10, 10, 100, 100, 100)
vis.AddLight(chrono.ChVector3D(-1, 1, 2), chrono.ChVector3D(0, 0, 0), True, True)
vis.SetLogoTexture(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.SetLogoPosition(chrono.ChVector3D(10, 10, 0))
vis.SetLogoScale(chrono.ChVector3D(0.5, 0.5, 1))


time = 0
while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    sys.DoStepDynamics(0.02)
    time += 0.02
    driver.Synchronize(time)
    vis.SetTimestep(0.02)
    vis.SetTryRealtime(True)