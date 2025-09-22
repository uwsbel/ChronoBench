import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


gator = veh.GatorVehicle(system)
gator.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
gator.SetContactMethod(veh.ChVehicleModel::ContactMethod::NSC)
gator.SetTireType(veh.TireModelType::TMEASY)
gator.SetChassisVisualizationType(veh.VisualizationType::MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType::MESH)
gator.SetWheelVisualizationType(veh.VisualizationType::MESH)


terrain = veh.RigidTerrain(system)
terrain.Initialize(system.GetContactMethod(), system.GetLcpSolverType(), 20, 20, 0, 0, 0, 10, 10)
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"))


driver = veh.ChInteractiveDriverIRR()
driver.Initialize(gator)


vis = chronoirr.ChIrrApp(system, "Gator Simulation", chronoirr.dimension2du(1024, 768))
vis.AddTypicalLogo()
vis.AddTypicalSky()
vis.AddTypicalCamera(chronoirr.vector3df(0, 3, -6))
vis.AddTypicalLights()
vis.AddTypicalEnvironment()
vis.AddLightWithShadow(chronoirr.vector3df(2, 10, -2), chronoirr.vector3df(0, 0, 0), 15, 5, 10, 40, 512, chronoirr.SColorf(0.8, 0.8, 1))
vis.AddShadowAll()
vis.AssetBindAll()
vis.AssetBindAll()
vis.AssetUpdateAll()


system.SetSolverType(chrono.ChSolver::Type::MINRES)
system.SetSolverWarmStarting(True)
system.SetMaxItersSolverSpeed(50)
system.SetMaxItersSolverStab(50)
system.SetTimestepperType(chrono.ChTimestepper::Type::HHT)
system.SetFixedStepSize(1e-3)


while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()
    vis.DoStep()
    vis.EndScene()
    system.DoStepDynamics(1e-3)
    driver.Synchronize(system)
    vis.SetTitle("Gator Simulation - Time: " + str(system.GetChTime()))