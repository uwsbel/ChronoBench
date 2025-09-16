import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


print('Copyright (c) 2023')


veh_sys = veh.ChVehicleSystem(veh.ChVehicleSystemType_VEHICLE_HMMWV)


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)


hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(-5, -5, 0.5), chrono.Q_from_AngZ(0)))
hmmwv.SetTireType(veh.TireType_TMEASY)
hmmwv.SetTireStepSize(1e-3)
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(hmmwv.GetSystem())
mesh_file = chrono.GetChronoDataFile("models/Highway/Highway_col.obj")
mesh = terrain.AddMesh(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0), mesh_file)
assert mesh
mesh->SetTexture(chrono.GetChronoDataFile("models/Highway/texture.png"), 100, 100)
mesh->SetContactSurfaceType(veh.ContactSurfaceType_FRICTION)
mesh->SetContactFrictionCoefficient(0.9)
mesh->SetContactRestitutionCoefficient(0.01)
mesh->SetContactMaterial(chrono.ChMaterialSurfaceSMC(0.9, 0.01, 0.0001, 2e8))
terrain.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachVehicle(hmmwv.GetVehicle())
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(10, 10, 5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(10, 20, 10), chrono.ChVector3d(0, 0, 0), 100, 10, 100, 120)
vis.EnableShadows()


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


while vis.Run():
    time = hmmwv.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize("", terrain)
    hmmwv.Advance(0.02)
    vis.Advance(0.02)
    vis.Render()