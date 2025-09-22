import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import os





chrono.SetChronoDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle'))


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1200, 800)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()


system.SetTimestep(0.01)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)






vehicle = chrono.ChVehicleModelHMMWV(system)
vehicle.SetChassisInitialPosition(chrono.ChVectorD(0, 0.5, 0))


vehicle.SetEngineType(chrono.ChVehicleModelHMMWV.EngineType_V8)
vehicle.SetDrivetrainType(chrono.ChVehicleModelHMMWV.DrivetrainType_AWD)


vehicle.SetChassisVisualizationType(chrono.ChVisualizationType_MESH)
vehicle.SetWheelVisualizationType(chrono.ChVisualizationType_MESH)


flat_patch1 = chrono.ChTerrainPatch(system)
flat_patch1.SetTexture(chrono.GetChronoDataPath() + '/textures/grass.png')
flat_patch1.SetSize(chrono.ChVectorD(10, 10))
flat_patch1.SetPosition(chrono.ChVectorD(-5, 0, -5))

flat_patch2 = chrono.ChTerrainPatch(system)
flat_patch2.SetTexture(chrono.GetChronoDataPath() + '/textures/asphalt.png')
flat_patch2.SetSize(chrono.ChVectorD(10, 10))
flat_patch2.SetRotation(chrono.Q_from_AngAxis(np.pi/4, chrono.ChVectorD(0, 1, 0)))
flat_patch2.SetPosition(chrono.ChVectorD(5, 0, -5))

bump_patch = chrono.ChTerrainPatchMesh(system)
bump_patch.SetMeshFilename(chrono.GetChronoDataPath() + '/meshes/bump.obj')
bump_patch.SetTexture(chrono.GetChronoDataPath() + '/textures/rock.png')
bump_patch.SetPosition(chrono.ChVectorD(0, 0, 0))

heightmap_patch = chrono.ChTerrainPatchHeightmap(system)
heightmap_patch.SetHeightmapFilename(chrono.GetChronoDataPath() + '/heightmaps/terrain.png')
heightmap_patch.SetTexture(chrono.GetChronoDataPath() + '/textures/dirt.png')
heightmap_patch.SetSize(chrono.ChVectorD(20, 20))
heightmap_patch.SetPosition(chrono.ChVectorD(10, 0, 10))





class Driver:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.steering = 0
        self.throttle = 0
        self.brake = 0

    def ApplyInputs(self):
        self.vehicle.SetSteeringAngle(self.steering)
        self.vehicle.SetThrottle(self.throttle)
        self.vehicle.SetBraking(self.brake)

driver = Driver(vehicle)

while vis.Run():
    
    if vis.GetDevice()->IsKeyDown(chrono.irr::KEY_LEFT):
        driver.steering = -1
    elif vis.GetDevice()->IsKeyDown(chrono.irr::KEY_RIGHT):
        driver.steering = 1
    else:
        driver.steering = 0

    if vis.GetDevice()->IsKeyDown(chrono.irr::KEY_W):
        driver.throttle = 1
    elif vis.GetDevice()->IsKeyDown(chrono.irr::KEY_S):
        driver.brake = 1
    else:
        driver.throttle = 0
        driver.brake = 0

    
    driver.ApplyInputs()

    
    system.DoStepDynamics(system.GetTimestep())

    
    vis.Render()