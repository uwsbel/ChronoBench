import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# Initialize the PyChrono environment and core components
chrono.SetChronoDataPath('./data/')

# Create the simulation system and add it to the environment
mysystem = chrono.ChSystemNSC()
chrono.ChCollisionSystem.SetDefaultImpulseThreshold(1e-3)

# Create the vehicle and add it to the system
hmmwv = veh.HMMWV()
hmmwv.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
hmmwv.SetChassisFixed(False)
mysystem.Add(hmmwv.GetChassisBody())
mysystem.Add(hmmwv.GetPart(0))
mysystem.Add(hmmwv.GetPart(1))
mysystem.Add(hmmwv.GetPart(2))
mysystem.Add(hmmwv.GetPart(3))
mysystem.Add(hmmwv.GetPart(4))
mysystem.Add(hmmwv.GetPart(5))
mysystem.Add(hmmwv.GetPart(6))
mysystem.Add(hmmwv.GetPart(7))

# Create the terrain and add it to the system
terrain = chrono.ChBodyEasyBox(mysystem, 10, 10, 1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
mysystem.Add(terrain)

# Create the driver and add it to the system
driver = veh.ChIrrGuiDriver()
driver.SetVehicle(hmmwv)
driver.SetSteeringMode(veh.SteeringMode::STEERING_MODE_TANK)
driver.SetSteeringCoeff(0.01)
driver.SetThrottleCoeff(0.01)
driver.SetBrakingCoeff(0.5)
mysystem.Add(driver.GetChassisBody())

# Create the IMU and GPS sensors and add them to the vehicle
imu = sens.ChSensorIMU(hmmwv.GetChassisBody())
imu.SetNoiseLevel(0.01)
imu.SetBias(chrono.ChVectorD(0, 0, 0))
imu.SetNoiseType(sens.SensorNoiseType::GAUSSIAN)
hmmwv.GetChassisBody().Add(imu)

gps = sens.ChSensorGPS(hmmwv.GetChassisBody())
gps.SetNoiseLevel(0.01)
gps.SetBias(chrono.ChVectorD(0, 0, 0))
gps.SetNoiseType(sens.SensorNoiseType::GAUSSIAN)
hmmwv.GetChassisBody().Add(gps)

# Create the visual interface using Irrlicht
application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(1024, 768)
application.SetWindowTitle('HMMWV Simulation')
application.SetTypicalLenght(1.0)
application.SetCamera(chrono.ChVectorD(0, 0, 1.5), chrono.ChVectorD(0, 0, 0))
application.AddTypicalLights()
application.AddSkyBox()
application.AddLogo('pychrono_logo.pvr')
application.AddCameraSkyBox()
application.AddCameraVerticalAlign(chrono.ChVectorD(0, 0, 0))
application.AddCameraHorizontalAlign(chrono.ChVectorD(0, 0, 0))
application.AddCameraSpeed(chrono.ChVectorD(0, 0, 0))
application.AddCameraAcceleration(chrono.ChVectorD(0, 0, 0))
application.AddCameraDampingRatio(0.5)
application.AddCameraNaturalFrequency(10.0)
application.AddCameraMode(chronoirr.CameraMode::CAMERA_MODE_FOLLOW_PATH)
application.AddCameraPathSpeed(0.5)
application.AddCameraPathMode(chronoirr.CameraPathMode::CAMERA_PATH_MODE_CIRCLE)
application.AddCameraPathRadius(1.0)
application.AddCameraPathCenter(chrono.ChVectorD(0, 0, 0))
application.AddCameraPathVerticalOffset(0.5)
application.AddCameraPathHorizontalOffset(0.5)
application.AddCameraPathAzimuth(0.0)
application.AddCameraPathElevation(0.0)
application.AddCameraPathRoll(0.0)

# Run the simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    application.DoStep()
    mysystem.DoStepDynamics(0.01)
    hmmwv.Update(0.01)
    driver.Update(0.01)
    imu.Process(0.01)
    gps.Process(0.01)
    print("Vehicle mass: ", hmmwv.GetChassisBody().GetMass())