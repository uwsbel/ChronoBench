import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


chrono.ChCollisionSystem.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionSystem.SetDefaultSuggestedMargin(0.001)


m113 = veh.M113()
m113.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
m113.SetChassisCollisionType(veh.ChassisCollisionType::MULTI_BODY)
m113.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngX(0)))
m113.Initialize()


sys.Add(m113.GetChassisBody())
sys.Add(m113.GetWheel1())
sys.Add(m113.GetWheel2())
sys.Add(m113.GetWheel3())
sys.Add(m113.GetWheel4())


terrain = veh.RigidTerrain(m113.GetSystem())
terrain_mat = chrono.ChMaterialSurfaceNSC()
terrain_mat.SetFriction(0.8)
terrain_mat.SetRestitution(0.5)
patch_mat = terrain_mat
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


sys.Add(terrain)


driver = veh.SimpleDriver()
driver.SetVehicle(m113)
driver.SetSteering(0)
driver.SetThrottle(0.5)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("M113 Simulation")
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))


vis.AttachSystem(sys)


time = 0
dt = 0.01
while vis.Run():
    
    sys.DoStepDynamics(dt)
    
    
    driver.Synchronize(time)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    time += dt