import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh


chrono.ChEngine.Initialize()


vis = irr.ChIrrApp()
vis.SetWindowSize(chrono.ChVector2i(800, 600))
vis.SetWindowTitle("HMMWV on Rigid Terrain")
vis.UseZoomBar(True)
vis.ShowLogo(False)


vehicle = veh.ChHMMWV()


vehicle.SetContactMethod(veh.ChContactMethod_TM)
vehicle.SetTireModel(veh.ChTireModel_TMEASY)
vehicle.SetTireStepSize(0.01)


vehicle.SetChassisBodyFixed(False)
vehicle.SetChassisInitialPosition(chrono.ChVector3d(0, 0, 1))
vehicle.SetChassisInitialRotation(chrono.ChQuaterniond(1, 0, 0, 0))


terrain = veh.ChRigidTerrain(vehicle.GetSystem())
terrain.SetTerrainDimensions(chrono.ChVector3d(10, 10, 0))
terrain.SetTexture(chrono.ChVector2d(10, 10), "chrono_textures/checker1.jpg")


vehicle.Initialize(terrain)
system = vehicle.GetSystem()


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetVehicle(vehicle)
driver.SetChaseCamDistance(10)
driver.SetChaseCamLag(0.5)
driver.SetChaseCamHeight(2)


system.SetTimestep(0.02)
system.SetCollisionSystem(chrono.ChCollisionSystemBruteForce())


while vis.Run() and not vis.WasKeyPressed(irr.KEY_ESCAPE):
    vis.BeginScene()
    vis.RenderItem(terrain)
    vis.RenderItem(vehicle)
    vis.EndScene()
    system.DoStepDynamics(0.02)
    driver.Update()