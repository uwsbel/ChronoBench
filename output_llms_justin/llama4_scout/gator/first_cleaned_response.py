import chrono as ch
import chrono.irrlicht as irr
import chrono.vehicle as veh


ch.ChEngine.Initialize()


app = irr.IrrlichtApplication()
app.SetTimestep(1 / 50.0)


gator = veh.Gator()


veh_params = veh.VehicleParams()
veh_params.SetContactMethod(veh.ContactMethod_NURBS)
veh_params.SetTireModel(veh.TireModel_TMEASY)
veh_params.SetTireStepSize(0.02)
veh_params.SetChassisVisualizationType(veh.VisualizationType_MESH)
veh_params.SetWheelVisualizationType(veh.VisualizationType_MESH)

gator.Initialize(veh_params)


gator.SetPos(ch.ChVector3d(0, 0, 1))
gator.SetRot(ch.ChQuaterniond(0, 0, 0, 1))


terrain = veh.RigidTerrain(gator.GetSystem())
terrain.SetTerrainSize(100, 100)
terrain.SetSampleDensity(1)
terrain.SetCoefficientFriction(0.8)
terrain.AddPatch(ch.ChVector3d(0, 0, 0), ch.ChVector3d(100, 0, 0), ch.ChVector3d(100, 100, 0), ch.ChVector3d(0, 100, 0), ch.ChColor(1, 1, 1))


terrain_mat = ch.ChMaterialSurfaceNSC()
terrain_mat.SetTexture(irr.GetChronoDataPath() + 'textures/terrain.jpg')


driver = veh.Driver(gator.GetSystem(), gator.GetChassisBody())
driver.SetSteering(veh.DriverSteering())
driver.SetThrottle(veh.DriverThrottle())
driver.SetBraking(veh.DriverBraking())


scene_node = app.AddMeshSceneNode(gator.GetChassisBody())


def Simulate(t):
    gator.Update()
    driver.Update()
    terrain.Update()
    app.Render()

app.SetSimulationFunction(Simulate)


app.Run()