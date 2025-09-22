import time

import chrono
from chrono import ChSystemSMC, ChBody, ChLinkMate, ChMaterialSurface, ChVector, ChMatrix33
from chrono.irrlicht import ChIrrApp
from chrono.vehicle import ChVehicle, ChChassis, ChTire, ChSteering, ChDriver, ChEngine, ChTransmission, ChDriveline, ChAxle, ChSuspension


my_system = ChSystemSMC()



terrain = my_system.Add(chrono.ChBody())
terrain.SetCollide(False)
terrain_shape = terrain.AddBox(100, 0.1, 100, chrono.ChMaterialSurface())
terrain_shape.SetFriction(0.5)


chassis = my_system.Add(ChChassis())
tire_FL = my_system.Add(ChTire())
tire_FR = my_system.Add(ChTire())
tire_RL = my_system.Add(ChTire())
tire_RR = my_system.Add(ChTire())

steering = ChSteering()
engine = ChEngine()
transmission = ChTransmission()
driveline = ChDriveline()
axle_FL = ChAxle()
axle_FR = ChAxle()
suspension_FL = ChSuspension()
suspension_FR = ChSuspension()

vehicle = ChVehicle()
vehicle.AddSubsystem(chassis)
vehicle.AddSubsystem(steering)
vehicle.AddSubsystem(engine)
vehicle.AddSubsystem(transmission)
vehicle.AddSubsystem(driveline)
vehicle.AddSubsystem(axle_FL)
vehicle.AddSubsystem(axle_FR)
vehicle.AddSubsystem(suspension_FL)
vehicle.AddSubsystem(suspension_FR)
vehicle.AddSubsystem(tire_FL)
vehicle.AddSubsystem(tire_FR)
vehicle.AddSubsystem(tire_RL)
vehicle.AddSubsystem(tire_RR)


chassis.SetPos(ChVector(0, 0, 0))
chassis.SetRot(ChMatrix33.ZYX_rotate(chrono.ChVector(0, 0, 0)))

tire_FL.SetPos(ChVector(-1.5, 0.5, 0))
tire_FR.SetPos(ChVector(1.5, 0.5, 0))
tire_RL.SetPos(ChVector(-1.5, -0.5, 0))
tire_RR.SetPos(ChVector(1.5, -0.5, 0))


imu = MyIMUSensor()  
gps = MyGPSSensor()  
chassis.AddSensor(imu)
chassis.AddSensor(gps)


vis = ChIrrApp(my_system, "HMMWV Simulation", 800, 600, False)
vis.AddLogo(chrono.GetChronoDataFile("logo_white_small.png"))
vis.AddTypicalSky()
vis.AddCamera(chrono.ChVector(5, 5, 5))
vis.AddLightPoint(chrono.ChVector(0, 10, 0), 0.5, 0.5, 0.5)

driver = ChDriver()
vehicle.SetDriver(driver)

vis.AddModule(driver)
vis.AddModule(vehicle)

while vis.Run():
    
    imu.Update()
    gps.Update()

    my_system.DoStepDynamics(1 / 60.0)

    vis.DrawAll()
    vis.WaitKey(5)

    print("Vehicle mass:", chassis.GetMass())


print("Final vehicle mass:", chassis.GetMass())
vis.GetDevice().drop()