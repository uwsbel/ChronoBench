import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.ChSystemSMC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on Complex Terrain')
vis.Initialize()


hmmwv = chrono.ChVehicleHMMWV()
hmmwv.SetChassisVisualizationType(chrono.ChVehicleVisualizationType.MESH)
hmmwv.SetSuspensionVisualizationType(chrono.ChVehicleVisualizationType.MESH)
hmmwv.SetWheelVisualizationType(chrono.ChVehicleVisualizationType.MESH)


hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0))))
hmmwv.SetEngineType(chrono.ChHMMWV_EngineType.SHAFT)
hmmwv.SetDriveType(chrono.ChHMMWV_DriveType.FOUR_WD)


terrain = chrono.ChBodyEasyBox(100, 100, 0.1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, 0, -0.05))
terrain.SetBodyFixed(True)


flat_patch_1 = chrono.ChBodyEasyBox(50, 50, 0.1, 1000, True, True)
flat_patch_1.SetPos(chrono.ChVectorD(-25, 0, -0.05))
flat_patch_1.SetTexture(chrono.GetChronoDataFile('textures/concrete.png'))

flat_patch_2 = chrono.ChBodyEasyBox(50, 50, 0.1, 1000, True, True)
flat_patch_2.SetPos(chrono.ChVectorD(25, 0, -0.05))
flat_patch_2.SetTexture(chrono.GetChronoDataFile('textures/grass.png'))


bump_mesh = chrono.ChTriangleMeshConnected()
bump_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('models/bump.obj'), True, True)
bump_body = chrono.ChBodyEasyMesh(bump_mesh, 1000, True, True)
bump_body.SetPos(chrono.ChVectorD(0, 0, 0.1))


heightmap_data = np.loadtxt(chrono.GetChronoDataFile('heightmaps/heightmap.txt'))
heightmap_body = chrono.ChBodyEasyHeightfield(heightmap_data, 0.1, 0.1, 0.1, 1000, True, True)
heightmap_body.SetPos(chrono.ChVectorD(0, 0, -0.05))


chrono.GetSystem().Add(terrain)
chrono.GetSystem().Add(flat_patch_1)
chrono.GetSystem().Add(flat_patch_2)
chrono.GetSystem().Add(bump_body)
chrono.GetSystem().Add(heightmap_body)


chrono.GetSystem().Add(hmmwv.GetVehicle())


driver = chrono.ChInteractiveDriver(hmmwv.GetVehicle())
driver.SetSteeringDelta(0.01)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)


timestep = 0.01


while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    driver.Update()

    
    chrono.GetSystem().DoStepDynamics(timestep)

    vis.EndScene()