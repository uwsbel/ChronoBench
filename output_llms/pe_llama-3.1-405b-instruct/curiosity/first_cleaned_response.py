import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()
contact_method = chrono.ChMaterialSurface::NSC
chassis_collision_type = veh.ChassisCollisionType::BOX


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground_shape = chrono.ChVisualShapeBox(100, 1, 100)
ground.AddVisualShape(ground_shape)
sys.Add(ground)


rover = veh.CuriosityRover()
rover.SetContactMethod(contact_method)
rover.SetChassisCollisionType(chassis_collision_type)
rover.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 1, 0), chrono.QUNIT))
rover.Initialize()


sys.Add(rover)


driver = veh.Rover_Driver()
rover.SetDriver(driver)


terrain = veh.RigidTerrain(rover.GetSystem())
patch = terrain.AddPatch(chrono.ChMaterialSurface::SMC, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)


vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, -10))
vis.AddTypicalLights()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()


while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    sys.DoStepDynamics(0.01)