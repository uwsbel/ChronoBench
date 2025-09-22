import chrono
from chrono.vehicle import ChVehicleModel, ChTireModelList, ChTire, ChRigidTire
from chrono.collision import ChCollisionModel
from chrono.irrlicht import ChIrrApp
from chrono.irrlicht.ChIrrODESolver import ChIrrODESolver
from chrono.irrlicht.user_force import ChDriver
from chrono.terrain import ChTerrain, ChTerrainForce, ChTerrainSCM


chrono.SetChDir(chrono.GetChronoDataPath())
chrono.SetShareThreads(True)


myapp = ChIrrApp(width=800, height=600, fullscreen=False, irrlicht_window_title="HMMWV on SCM Terrain")


vehicle = ChVehicleModel()
vehicle.SetChassisGeometryType(ChVehicleModel.CHASSIS_BOX)
vehicle.SetChassis(chrono.ChBodyEasyBox(0.5, 0.25, 2.0, 1000, chrono.ChMaterialSurface.MATERIAL_METAL))
vehicle.SetChassisCollisionType(ChVehicleModel.CHASSIS_BOX_COLLISION)
vehicle.SetChassisVisualizationType(ChVehicleModel.CHASSIS_BOX_VISUALIZATION)


tire_model = ChTireModelList()
tire_model.AddTireModel(ChTire.RIGID)
tire_model.AddVisualizationType(ChTire.RIGID, ChTire.MESH)


vehicle.AddTire(tire_model, 0.5, 0.2, -1.0, 0.5, 0.2, -1.0, 1.0)
vehicle.AddTire(tire_model, 0.5, 0.2, 1.0, 0.5, 0.2, 1.0, 1.0)
vehicle.AddTire(tire_model, -0.5, 0.2, -1.0, -0.5, 0.2, -1.0, 1.0)
vehicle.AddTire(tire_model, -0.5, 0.2, 1.0, -0.5, 0.2, 1.0, 1.0)


terrain = ChTerrain()
terrain_params = ChTerrain.SCM_TERRAIN_FORCE_GENERATOR_PARAMETERS()
terrain_params.soil.density = 1500  
terrain_params.soil.static_friction = 0.6  
terrain_params.soil.dynamic_friction = 0.4  
terrain_params.soil.restitution = 0.1  
terrain_params.soil.cohesion = 100  
terrain_params.soil.viscous_coefficient = 0.0  
terrain_params.soil.young_modulus = 5e6  
terrain_params.soil.poisson_ratio = 0.3  
terrain_params.soil.tension_limit = 1e4  
terrain_params.soil.compression_limit = 1e4  
terrain_params.soil.internal_friction = 0.5  
terrain_params.soil.external_friction = 0.5  

terrain.SetTerrainForceGenerator(ChTerrainForce.SCM_TERRAIN_FORCE_GENERATOR, terrain_params)
terrain.SetVisualizationType(ChTerrain.VISUALIZATION_TYPE_DISPLACEMENT_COLOR)


terrain.SetMovingPatch(True)


driver = ChDriver()
vehicle.SetDriver(driver)


myapp.AddModule(ChIrrODESolver(1.0 / 50.0))


myapp.AddAssetsFolder(chrono.GetChronoDataPath() + "/irrlicht/media")
myapp.AddModule(vehicle)
myapp.AddModule(terrain)


myapp.SetTimestep(1.0 / 50.0)
myapp.SubscribeToClose(myapp.GetSystem(), "SimEnd")


myapp.Initialize()
myapp.SetUp()
while myapp.GetDevice().run():
    myapp.GetODESolver().Solve()
    myapp.Render()
myapp.Finalize()