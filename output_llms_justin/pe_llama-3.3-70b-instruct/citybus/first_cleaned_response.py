import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()
contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.9)
contact_material.SetDampingF(0.000)
contact_material.SetCompliance(0.0001)
contact_material.SetDampingT(0.000)


vehicle = veh.CityBus()
vehicle.SetContactMaterial(contact_material)
vehicle.SetChassisCollisionType(chrono.ChCollisionType.BULLET)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_PI / 2)))
vehicle.Initialize()


terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.SetContactMaterial(contact_material)
patch_mat = veh.ChMaterialTire()
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


driver = veh.ChIrrlichtDriver()
vehicle.AddDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("CityBus Simulation")
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddSkyBox()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddTypicalLights()
vis.AttachSystem(sys)


time_step = 0.02
while vis.Run():
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()