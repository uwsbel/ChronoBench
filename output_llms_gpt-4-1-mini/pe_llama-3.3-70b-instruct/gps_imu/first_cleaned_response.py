import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


veh_hmmwv = veh.HMMWV()
veh_hmmwv.SetContactMaterial(chrono.ChContactMaterialSurfaceNSC())
veh_hmmwv.SetChassisCollisionType(chrono.ChCollisionType.BULLET)
veh_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngX(chrono.CH_PI_2)))
veh_hmmwv.Initialize()
veh_hmmwv.AddDriver(veh.FullDriver())
sys.Add(veh_hmmwv)


terrain = veh.RigidTerrain(veh_hmmwv.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 10, 10)
terrain.Initialize()
sys.Add(terrain)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AttachSystem(sys)


imu_sensor = veh.IMU_Sensor(veh_hmmwv.GetChassis())
gps_sensor = veh.GPS_Sensor(veh_hmmwv.GetChassis())
veh_hmmwv.AddSensor(imu_sensor)
veh_hmmwv.AddSensor(gps_sensor)


time_step = 0.01
while vis.Run():
    
    sys.DoStepDynamics(time_step)
    
    
    imu_sensor.Update()
    gps_sensor.Update()
    
    
    print("Vehicle mass: ", veh_hmmwv.GetMass())
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()