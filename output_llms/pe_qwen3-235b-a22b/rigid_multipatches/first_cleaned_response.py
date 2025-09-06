import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), '..', 'data'))
veh.SetDataPath(os.path.join(os.path.dirname(__file__), '..', 'data', 'vehicle'))


sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_MESH)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.5), chrono.QuatFromAngleZ(0)))
vehicle.SetEngineType(veh.EngineModelType_SIMPLE)
vehicle.SetDrivetrainType(veh.DrivetrainType_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(0.005)
vehicle.Initialize()


terrain = veh.RigidTerrain(vehicle.GetSystem())


patch_mat1 = chrono.ChContactMaterialSMC()
patch_mat1.SetFriction(0.9)
patch_mat1.SetRestitution(0.2)
patch1 = terrain.AddPatch(patch_mat1, chrono.ChCoordsysd(), 100, 100)
patch1.SetTexture(veh.GetDataFile("terrain/textures/asphalt.png"), 200, 200)


patch_mat2 = chrono.ChContactMaterialSMC()
patch_mat2.SetFriction(0.8)
patch_mat2.SetRestitution(0.3)
bump_coords = chrono.ChCoordsysd(chrono.ChVector3d(20, 0, 0), chrono.QuatFromAngleY(0))
patch2 = terrain.AddPatch(patch_mat2, bump_coords, 10, 10)
patch2.SetMesh(veh.GetDataFile("terrain/meshes/bump.obj"), 0.1)


patch_mat3 = chrono.ChContactMaterialSMC()
patch_mat3.SetFriction(0.7)
patch_mat3.SetRestitution(0.4)
heightmap_coords = chrono.ChCoordsysd(chrono.ChVector3d(-20, 0, 0), chrono.QuatFromAngleY(0))
patch3 = terrain.AddPatch(patch_mat3, heightmap_coords, 20, 20)
patch3.SetHeightmap(veh.GetDataFile("terrain/heightmaps/heightmap.png"), 20, 20, 0, 2)
patch3.SetTexture(veh.GetDataFile("terrain/textures/grass.png"), 200, 200)

terrain.Initialize()


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.06)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


step_size = 0.01
while vis.Run():
    time = sys.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    
    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    
    sys.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()