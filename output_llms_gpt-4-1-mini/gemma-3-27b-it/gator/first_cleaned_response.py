import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh













sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)


terrain = veh.RigidTerrain(sys)
terrain_length = 100
terrain_width = 100
terrain.AddPatch(chrono.ChLinkMaterialSurfaceSMC(),
                 chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_PI_2, chrono.ChVector3d(1, 0, 0))),
                 terrain_length, terrain_width)
terrain.Initialize()


gator = veh.Gator(sys)
gator.SetContactMethod(veh.ChContactMethod.SMC)
gator.SetChassisCollisionType(veh.ChChassis.CollisionType.MESH)
gator.SetTireModel(veh.TireModel.TMEASY)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))))
gator.Initialize()


for body in gator.GetVehicle().GetBodies():
    if body.GetVisualShape(0) is None:
        body.AddVisualShape(chrono.ChVisualShapeMesh(chrono.GetChronoDataFile("vehicle/gator/meshes/chassis.obj")))
    body.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))  

for tire in gator.GetVehicle().GetTires():
    if tire.GetVisualShape(0) is None:
        tire.AddVisualShape(chrono.ChVisualShapeMesh(chrono.GetChronoDataFile("vehicle/gator/meshes/tire.obj")))
    tire.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.2))  



driver = veh.GatorInteractiveDriver(gator)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)







vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, -10))
vis.AddTypicalLights()







while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver.Synchronize(sys.GetChTime())

    
    sys.DoStepDynamics(0.01)