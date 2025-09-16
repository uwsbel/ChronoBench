import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np







my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


myapplication = chronoirr.ChIrrApp(my_system, "HMMWV on Complex Terrain", chronoirr.dimension2du(1280, 720))


myapplication.AddTypicalCamera(chronoirr.vector3df(0, 10, -20), chronoirr.vector3df(0, 0, 0))


terrain = chrono.ChTerrainModel()
terrain.SetGroundTexture(chrono.GetChronoDataPath() + 'terrain/textures/grass.png')


patch1 = chrono.ChTerrainPatch()
patch1.SetSize(chrono.ChVectorD(100, 100))
patch1.SetTexture(chrono.GetChronoDataPath() + 'terrain/textures/rock.png')
patch1.SetHeight(0)
terrain.AddPatch(patch1, chrono.ChVectorD(0, 0, 0))

patch2 = chrono.ChTerrainPatch()
patch2.SetSize(chrono.ChVectorD(50, 50))
patch2.SetTexture(chrono.GetChronoDataPath() + 'terrain/textures/sand.png')
patch2.SetHeight(2)
terrain.AddPatch(patch2, chrono.ChVectorD(50, 50, 0))


bump_mesh = chrono.ChTriangleMeshConnected()
bump_mesh.LoadFromFile(chrono.GetChronoDataPath() + 'terrain/meshes/bump.obj')
bump_patch = chrono.ChTerrainPatch()
bump_patch.SetMesh(bump_mesh)
bump_patch.SetHeight(1)
terrain.AddPatch(bump_patch, chrono.ChVectorD(100, 0, 0))


heightmap = chrono.ChHeightmap()
heightmap.LoadHeightmap(chrono.GetChronoDataPath() + 'terrain/heightmaps/terrain.png')
heightmap_patch = chrono.ChTerrainPatch()
heightmap_patch.SetHeightmap(heightmap)
terrain.AddPatch(heightmap_patch, chrono.ChVectorD(150, 0, 0))


my_system.Add(terrain)







vehicle = chrono.ChVehicleModel()
vehicle.SetChassisCollision(True)
vehicle.SetChassisMaterial(chrono.ChMaterialSurfaceNSC(0.9, 0.9, 0.1))
vehicle.SetChassisFixed(False)
vehicle.SetChassisBody(chrono.ChBodyEasyBox(2.5, 1.2, 0.8, 1000))
vehicle.SetInitPosition(chrono.ChVectorD(0, 1.5, 0))


vehicle.AddWheel(chrono.ChWheelInfo(
    position=chrono.ChVectorD(-1.2, -0.6, 0.4),
    radius=0.3,
    mass=20,
    inertia=0.1,
    suspension_spring=5000,
    suspension_damping=500,
    friction=2.0
))
vehicle.AddWheel(chrono.ChWheelInfo(
    position=chrono.ChVectorD(1.2, -0.6, 0.4),
    radius=0.3,
    mass=20,
    inertia=0.1,
    suspension_spring=5000,
    suspension_damping=500,
    friction=2.0
))


vehicle.SetEngine(chrono.ChEngineSimple(10000, 2000))
vehicle.SetDrivetrainType(chrono.ChVehicleModel.DRIVETRAIN_RWD)


my_system.Add(vehicle)







driver = chrono.ChDriverSteering(vehicle)
driver.SetSteeringGain(1.5)
driver.SetThrottleGain(0.5)
driver.SetBrakingGain(1.0)







myapplication.SetTimestep(0.01)
myapplication.SetStepManage(True)

while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    
    
    steering = myapplication.GetDevice().GetMouseWheel()
    throttle = myapplication.GetDevice().GetKeyboard()->GetKeyCode(chrono.irr::KEY_UP) - myapplication.GetDevice().GetKeyboard()->GetKeyCode(chrono.irr::KEY_DOWN)
    brake = myapplication.GetDevice().GetKeyboard()->GetKeyCode(chrono.irr::KEY_SPACE)

    driver.SetSteeringInput(steering)
    driver.SetThrottleInput(throttle)
    driver.SetBrakingInput(brake)

    
    my_system.DoStepDynamics(myapplication.GetTimestep())

    myapplication.EndScene()