import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemSMC()


veh_hmmwv = veh.HMMWV()
veh_hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
veh_hmmwv.SetChassisCollisionType(chrono.ChCollisionType_PRIMITIVE)
veh_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.2), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
veh_hmmwv.Initialize()


sys.Add(veh_hmmwv.GetPart(0))


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()
vis.AttachSystem(sys)
vis.Initialize()


driver = veh.CreateDriverHMMWV(veh_hmmwv)
driver.SetSteeringGain(0.5)
driver.SetThrottleGain(0.5)
driver.SetBrakingGain(0.5)


while vis.Run():
    
    sys.DoStepDynamics(0.02)
    veh_hmmwv.GetChassis().DoStepDynamics(0.02)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    driver.Synchronize(0.02)
    
    
    chrono.ChSleep(0.02)