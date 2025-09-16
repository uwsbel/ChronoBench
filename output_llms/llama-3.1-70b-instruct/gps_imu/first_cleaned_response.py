import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle
import pychrono.sensor as chrono_sensor


chrono.SetChronoDataPath('./data/')
system = chrono.ChSystemNSC()


veh = chrono_vehicle.HMMWV()
veh.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
veh.SetChassisMaterial(chrono.ChMaterialSurfaceNSC())
veh.SetTireMaterial(chrono.ChMaterialSurfaceNSC())
veh.SetTireFrictionCoefficient(0.9)
veh.SetTireDampingCoefficient(0.9)
veh.SetChassisVisualization(chrono.ChVisualizationType.PRIMITIVES)
veh.SetTireVisualization(chrono.ChVisualizationType.MESH)
veh.SetTireSmoothMesh(True)
veh.SetChassisCollisionType(chrono.ChCollisionType.BULLET)
veh.SetTireCollisionType(chrono.ChCollisionType.BULLET)
veh.SetChassisSuggestedEnvelope(chrono.ChVectorD(1, 1, 1))
veh.SetTireSuggestedEnvelope(chrono.ChVectorD(1, 1, 1))
veh.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))


system.Add(veh.GetChassisBody())
system.Add(veh.GetBodyList())


terrain = chrono.ChBodyEasyBox(system, 10, 1, 10, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetFriction(0.9)
terrain.SetDamping(0.9)
system.Add(terrain)


driver = chrono_vehicle.ChIrrVehicleDriver(veh)
driver.SetSteeringDelta(chrono.CH_C_PI/180)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.1)
driver.SetTractionLag(0.2)
driver.SetSteeringLag(0.2)
driver.SetThrottleLag(0.2)
driver.SetBrakingLag(0.2)
driver.Initialize()


imu = chrono_sensor.ChSensorIMU(veh.GetChassisBody())
imu.SetNoiseAcc(0.1)
imu.SetNoiseGyr(0.1)
imu.SetNoiseMag(0.1)
imu.SetFrequency(100)
imu.SetPosition(chrono.ChVectorD(0, 0, 0))
imu.SetOrientation(chrono.Q_from_AngX(chrono.CH_C_PI_2))
imu.Initialize()

gps = chrono_sensor.ChSensorGPS(veh.GetChassisBody())
gps.SetNoisePos(0.1)
gps.SetNoiseVel(0.1)
gps.SetFrequency(100)
gps.SetPosition(chrono.ChVectorD(0, 0, 0))
gps.SetOrientation(chrono.Q_from_AngX(chrono.CH_C_PI_2))
gps.Initialize()


application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(1024, 768)
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.ChVectorD(0, 1, -2))
application.AddSkyBox()
application.AddLogo()
application.SetShowInfo(True)
application.SetShowLogo(True)
application.SetCamera(chronoirr.ChVectorD(0, 1, -2))


steering = 0
throttle = 0
braking = 0
while application.Run():
    application.BeginScene()
    application.DrawAll()
    
    
    driver.SetSteering(steering)
    driver.SetThrottle(throttle)
    driver.SetBraking(braking)
    
    
    veh.Update(1e-3)
    
    
    terrain.Update(1e-3)
    
    
    imu.Update(1e-3)
    gps.Update(1e-3)
    
    
    print("Vehicle mass: ", veh.GetChassisBody().GetMass())
    
    
    system.DoStepDynamics(1e-3)
    
    
    application.EndScene()
    
    
    steering += 0.01
    throttle += 0.01
    braking += 0.01
    
    
    steering = max(-chrono.CH_C_PI, min(chrono.CH_C_PI, steering))
    throttle = max(0, min(1, throttle))
    braking = max(0, min(1, braking))