import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemSMC()


gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(chrono.ChCollisionType_Mesh)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
gator.SetTireType(veh.TireType_TMEASY)
gator.SetTireRadius(0.6)
gator.Initialize(sys)


gator.SetVisualSystem(chronoirr.ChVisualSystemIrrlicht())
gator.AddVisualMesh(chrono.GetChronoDataFile("gator_mesh.obj"))


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


terrain_visual = chronoirr.ChVisualSystemIrrlicht()
terrain_visual.AddSkyBox()
terrain_visual.AddTypicalLights()
terrain_visual.SetWindowSize(1024, 768)
terrain_visual.SetWindowTitle("Gator Simulation")
terrain_visual.AddCamera(chrono.ChVectorD(0, 10, 20))
terrain_visual.AddTypicalLights()


driver = veh.InteractiveDriver(gator)


while terrain_visual.Run():
    sys.DoStepDynamics(0.02)
    terrain_visual.BeginScene()
    terrain_visual.DrawAll()
    terrain_visual.EndScene()
    terrain_visual.DrawAll()
    driver.Synchronize(1 / 50.0)