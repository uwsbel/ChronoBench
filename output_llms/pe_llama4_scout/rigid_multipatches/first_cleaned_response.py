import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.robot as robosimian


chrono.SetChronoDataPath('/path/to/chrono/data')


system = chrono.ChSystemSMC()


system.SetContactMethod(chrono.ChContactMethod.SMC)


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on Complex Terrain')
vis.Initialize()


hmmwv = veh.HMMWV()


hmmwv.SetContactMethod(chrono.ChContactMethod.SMC)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType.MESH)


init_pos = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleX(chrono.CH_PI / 2))
hmmwv.Initialize(init_pos)


hmmwv.SetEngineType(veh.EngineType.SIMPLE)
hmmwv.SetDrivetrainType(veh.DrivetrainType.AWD)


hmmwv.EnableVisualization(True)


system.Add(hmmwv.GetChassisBody())
for wheel in hmmwv.GetWheels():
    system.Add(wheel)
for link in hmmwv.GetLinks():
    system.Add(link)


terrain = veh.RigidTerrain(system)


patch_mat1 = chrono.ChMaterialSurfaceSMC()
patch_mat1.SetKf(1e4)
patch_mat1.SetKt(1e4)
patch_mat1.SetFriction(0.9)
patch1 = terrain.AddPatch(patch_mat1, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 10, 10)
patch1.SetTexture(chrono.GetChronoDataFile('textures/blue.jpg'))

patch_mat2 = chrono.ChMaterialSurfaceSMC()
patch_mat2.SetKf(1e4)
patch_mat2.SetKt(1e4)
patch_mat2.SetFriction(0.9)
patch2 = terrain.AddPatch(patch_mat2, chrono.ChCoordsysd(chrono.ChVector3d(10, 0, 0), chrono.QUNIT), 10, 10)
patch2.SetTexture(chrono.GetChronoDataFile('textures/red.jpg'))


bump_mesh = chrono.ChTriangleMeshConnected()
bump_mesh.LoadWavefrontObj(chrono.GetChronoDataFile('meshes/bump.obj'))
bump_mat = chrono.ChMaterialSurfaceSMC()
bump_mat.SetKf(1e4)
bump_mat.SetKt(1e4)
bump_mat.SetFriction(0.9)
terrain.AddMesh(bump_mesh, bump_mat, chrono.ChCoordsysd(chrono.ChVector3d(20, 0, 0), chrono.QUNIT))


heightmap = veh.HeightmapTerrain(100, 100, 1)
heightmap_mat = chrono.ChMaterialSurfaceSMC()
heightmap_mat.SetKf(1e4)
heightmap_mat.SetKt(1e4)
heightmap_mat.SetFriction(0.9)
terrain.AddHeightmap(heightmap, heightmap_mat, chrono.ChCoordsysd(chrono.ChVector3d(30, 0, 0), chrono.QUNIT))


driver = veh.Driver(hmmwv)


steering_input = chrono.ChFunction_Const(0.1)
throttle_input = chrono.ChFunction_Const(0.5)
braking_input = chrono.ChFunction_Const(0.0)


def UpdateDriver(t):
    steering = steering_input.Get_y(t)
    throttle = throttle_input.Get_y(t)
    braking = braking_input.Get_y(t)
    driver.SetInputs(steering, throttle, braking)


timestep = 0.01
while vis.Run():
    system.DoStepDynamics(timestep)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    UpdateDriver(system.GetChTime())