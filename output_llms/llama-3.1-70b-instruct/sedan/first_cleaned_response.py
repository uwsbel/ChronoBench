import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle
import numpy as np


mysystem = chrono.ChSystemNSC()
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)
mysystem.SetSolverType(chrono.ChSolver.Type_PSSOR)
mysystem.SetSolverMaxIterations(100)
mysystem.SetSolverForceTolerance(1e-12)


bmw = chrono_vehicle.ChVehicle()
bmw.SetContactMethod(chrono.ChContactMethod.NSC)
bmw.SetChassisCollisionModel(chrono.ChCollisionModel.MULTI_SHAPE)
bmw.SetChassisSimplified(false)
bmw.SetTireType(chrono_vehicle.ChTire.TMEASY)
bmw.SetTireFrictionCoefficient(0.9)
bmw.SetTireRollingRadius(0.3)
bmw.SetTireViscosity(0.8)


body = chrono.ChBodyEasyBox(bmw.GetChassis(), 1.8, 1.5, 0.5, 7800, True, True, chrono.ChVectorD(0, 0, 0.5))
bmw.SetChassisBody(body)


bmw.AddSteering(chrono.ChSteeringPerfect())
bmw.AddSuspension(chrono.ChSuspensionMaxwell(1.5, 2.5, 0.1))


bmw.AddTire(chrono.ChTireTMEasy(chrono.ChVectorD(-1.1, 0.7, 0.2), chrono.ChVectorD(1.1, 0.7, 0.2), 
                                chrono.ChVectorD(-1.1, -0.7, 0.2), chrono.ChVectorD(1.1, -0.7, 0.2)))


driver = chrono.ChIrrlichtDriver()
driver.SetVehicle(bmw)
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)


terrain = chrono.ChBodyEasyBox(mysystem, 10, 10, 0.5, 1000, True, True, chrono.ChVectorD(0, 0, -0.25))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetCollide(True)


mysystem.Add(bmw.GetChassis())
mysystem.Add(terrain)


application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(1024, 768)
application.AddSkyBox()
application.AddLogo(0.5, 0.5, "logo.png")
application.AddCamera(chrono.ChVectorD(0, 0, 1.5), chrono.ChVectorD(0, 0, 0))
application.AddTypicalLights()


irr_driver = chronoirr.ChIrrlichtDriver(application)
irr_driver.SetCamera(chrono.ChVectorD(0, 0, 1.5), chrono.ChVectorD(0, 0, 0))
irr_driver.SetSkyBox(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 1000)
irr_driver.SetLogo(chrono.ChVectorD(0.5, 0.5, 0), chrono.ChVectorD(0, 0, 1), 0.5)


dt = 0.01
t_end = 10


while mysystem.GetChTime() < t_end:
    mysystem.DoStepDynamics(dt)
    application.DoStepDynamics(dt)
    irr_driver.DoStepDynamics(dt)


application.Run()