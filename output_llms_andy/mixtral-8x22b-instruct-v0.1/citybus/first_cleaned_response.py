import PyChrono as chrono
import PyChrono.Irrlicht as chronoirr
import PyChrono.Vehicle as veh
import numpy as np


my_application = chronoirr.ChIrrApp(chronoirr.NullMaterial(), chronoirr.CollisionSystemType.CHRONO_COLLISION_SYSTEM_TYPE_FAST)
my_application.AddTypicalLogo(chronoirr.NullMaterial())
my_application.AddTypicalSky(chronoirr.NullMaterial())
my_application.AddTypicalLights(chronoirr.NullMaterial())
my_application.AddTypicalCamera(chronoirr.Vec3(0, 0, 10), chronoirr.Vec3(0, 0, 0))
my_application.SetTimestep(0.02)


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = chrono.ChTerrain(chrono.GetChronoDataFile('terrain/heightmap.tif'))
terrain.SetTexture(chrono.GetChronoDataFile('terrain/grass.jpg'))
terrain.SetContactMaterial(3e7, 0.4)
terrain.Initialize(my_system)


vehicle = veh.ChVehicle()
vehicle.SetChassis(veh.ChBodyEasyChassis(veh.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))))


tire_model = veh.ChTMeasyTire()
tire_model.Set_K_phi(1e5)
tire_model.Set_D_phi(1e3)
tire_model.Set_K_gamma(1e5)
tire_model.Set_D_gamma(1e3)
tire_model.Set_mu_x(1.0)
tire_model.Set_mu_y(1.2)
tire_model.Set_mu_spin(0.01)

for i in range(4):
    wheel = veh.ChWheel(vehicle, veh.ChVectorD(0, 0, 0), veh.ChVectorD(0, 0, 0), veh.ChVectorD(0.3, 0.5, 0.3), tire_model)
    wheel.SetSuspensionStiffness(30000)
    wheel.SetSuspensionDamping(1500)
    wheel.SetMaxSuspensionTravelCm(15)
    wheel.SetSuspensionForceAppPoint(veh.ChVectorD(0, 0, 0))
    wheel.SetSuspensionForceAppPoint(veh.ChVectorD(0, 0, 0))
    wheel.SetSuspensionForceAppPoint(veh.ChVectorD(0, 0, 0))
    wheel.SetTireForceAppPoint(veh.ChVectorD(0, 0, 0))
    wheel.SetSteering(veh.ChLinkSteering(vehicle, veh.ChVectorD(0, 0, 0), veh.ChVectorD(0, 0, 0)))
    wheel.SetBraking(veh.ChLinkBraking(vehicle, veh.ChVectorD(0, 0, 0), veh.ChVectorD(0, 0, 0)))
    wheel.SetTireModel(tire_model)
    vehicle.AddWheel(wheel)


vehicle.Initialize(my_system)


terrain.SetContactMaterial(3e7, 0.4)
terrain.Initialize(my_system)


driver = veh.ChIrrGuiDriver(my_application, my_system)
driver.SetSteeringDelta(0.04)
driver.SetSteeringOffset(0)
driver.SetMaxSteering(0.6)
driver.SetMaxSpeed(50)
driver.SetMaxAcceleration(20)
driver.SetMaxBraking(20)
driver.SetMaxReverseSpeed(10)
driver.SetMaxReverseAcceleration(10)
driver.SetMaxReverseBraking(10)


while my_application.GetDevice().run():
    my_application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    my_application.DrawAll()
    my_application.DoStep()
    my_application.EndScene()

my_application.Close()