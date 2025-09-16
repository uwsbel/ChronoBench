import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetTireModel(veh.TireModel_TMEASY)
hmmwv.SetChassisVisualization(chrono.ChVisualSystem().Asset("hmmwv_chassis.obj"))
hmmwv.SetSuspensionVisualization(chrono.ChVisualSystem().Asset("hmmwv_suspension.obj"))
hmmwv.SetSteeringVisualization(chrono.ChVisualSystem().Asset("hmmwv_steering.obj"))
hmmwv.SetWheelVisualization(chrono.ChVisualSystem().Asset("hmmwv_wheel.obj"))
hmmwv.SetLocation(chrono.ChVectorD(0, 0, 1))
hmmwv.SetRotation(chrono.Q_from_AngX(chrono.CH_C_PI_2))


terrain = chrono.ChBodyEasyMesh()
terrain.SetMesh(chrono.GetMeshList()[0])
terrain.SetCollide(True)
terrain.SetBodyFixed(True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.AddVisualMesh(chrono.GetMeshList()[1])
sys.Add(terrain)


sys.Add(hmmwv.GetPartChassis())
sys.Add(hmmwv.GetPartWheel1())
sys.Add(hmmwv.GetPartWheel2())
sys.Add(hmmwv.GetPartWheel3())
sys.Add(hmmwv.GetPartWheel4())


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(hmmwv)
driver.SetSteeringMode(veh.SteeringMode_KINEMATIC)
driver.SetThrottleMode(veh.ThrottleMode_PERCENT)
driver.SetBrakingMode(veh.BrakingMode_PERCENT)


step_size = 0.02
vis.SetWindowSize(1024, 768)
vis.SetCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))
vis.SetLogo("")

while vis.Run():
    
    hmmwv.Update(0.02)
    sys.DoStepDynamics(step_size)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    chrono.ChSleep(0.02)