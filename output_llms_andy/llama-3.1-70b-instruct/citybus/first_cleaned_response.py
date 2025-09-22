import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np


chrono.SetChronoDataPath('./data/')


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.ChCityBus()
vehicle.SetChassisFixed(False)
vehicle.SetChassisMass(1500)
vehicle.SetChassisInertiaXX(chrono.ChVectorD(100, 100, 100))
vehicle.SetChassisInertiaXY(chrono.ChVectorD(0, 0, 0))
vehicle.SetChassisInertiaXZ(chrono.ChVectorD(0, 0, 0))
vehicle.SetChassisInertiaYY(chrono.ChVectorD(100, 100, 100))
vehicle.SetChassisInertiaYZ(chrono.ChVectorD(0, 0, 0))
vehicle.SetChassisInertiaZZ(chrono.ChVectorD(100, 100, 100))
vehicle.SetChassisPosition(chrono.ChVectorD(0, 0, 0.5))
vehicle.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.SetTireModel(veh.TireModelType.TM_RIGID)


terrain = veh.RigidTerrain(system)
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile2.jpg"))
terrain.SetPlane(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(vehicle)
driver.SetSteeringMode(veh.SteeringModeType.SMT_POSITION)
driver.SetThrottleMode(veh.ThrottleModeType.TMT_POSITION)
driver.SetBrakingMode(veh.BrakingModeType.BMT_POSITION)
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)


app = chronoirr.ChVisualSystemIrrlicht(system)
app.AddTypicalLights()
app.AddTypicalSky()
app.AddTypicalLogo()
app.AddCamera(chrono.ChVectorD(0, 0, 1.5), chrono.ChVectorD(0, 0, 0))
app.SetCameraFollow(vehicle.GetChassisBody(), 1.5, 0.5)
app.SetWindowSize(1280, 720)
app.SetWindowTitle("CityBus Simulation")


step_size = 0.02
t_end = 10
steps = int(t_end / step_size)


for i in range(steps):
    system.DoStepDynamics(step_size)
    app.BeginScene(True, True, chrono.ChVectorD(0.5, 0.5, 0.5))
    app.DrawAll()
    app.EndScene()
    app.DoStep()
    driver.InputSteeringAndThrottle()
    driver.Advance(step_size)


app.GetSystem().DoStepDynamics(step_size)