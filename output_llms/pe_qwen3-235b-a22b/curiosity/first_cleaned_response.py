import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
patch.SetTexture(chrono.GetChronoDataFile("textures/terrain/red_clay.jpg"))
terrain.Initialize()


rover = veh.Curiosity(sys)
rover.SetChassisCollisionType(veh.Curiosity.CollisionType_ALL)
rover.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
rover.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, -10))
vis.AddTypicalLights()
vis.EnableShadows()

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
rover.SetDriver(driver)


time_step = 0.01
while vis.Run():
    
    driver.Synchronize(sys.GetChTime())
    
    
    sys.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()