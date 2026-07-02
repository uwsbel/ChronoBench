import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


vehicle = veh.BMW_E90()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


tire_left_front = veh.TMeasyTire("left_front")
tire_left_front.LoadTMeasyParams(os.path.join(veh.GetDataPath(), "tire", "tire_BMW_E90_left_front.json"))
tire_right_front = veh.TMeasyTire("right_front")
tire_right_front.LoadTMeasyParams(os.path.join(veh.GetDataPath(), "tire", "tire_BMW_E90_right_front.json"))
tire_left_rear = veh.TMeasyTire("left_rear")
tire_left_rear.LoadTMeasyParams(os.path.join(veh.GetDataPath(), "tire", "tire_BMW_E90_left_rear.json"))
tire_right_rear = veh.TMeasyTire("right_rear")
tire_right_rear.LoadTMeasyParams(os.path.join(veh.GetDataPath(), "tire", "tire_BMW_E90_right_rear.json"))

vehicle.GetLeftFrontWheel().SetTire(tire_left_front)
vehicle.GetRightFrontWheel().SetTire(tire_right_front)
vehicle.GetLeftRearWheel().SetTire(tire_left_rear)
vehicle.GetRightRearWheel().SetTire(tire_right_rear)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetFriction(0.9)
patch.SetRestitution(0.01)
terrain.Initialize()


logo = irr.IrrlichtDevice.getVideoDriver().getTexture(veh.GetDataFile("terrain/logos/bmw.png").c_str())
patch.AddLogo(logo, 0.5, 0.5, 10, 10)


driver = veh.ChInteractiveDriverIRR(vehicle)
driver.SetSteeringClamp(1.0)
driver.SetThrottleClamp(1.0)
driver.SetBrakingClamp(1.0)
driver.SetSteeringIncrement(0.02)
driver.SetThrottleIncrement(0.02)
driver.SetBrakingIncrement(0.06)


app = irr.ChIrrApp(vehicle.GetSystem(), "BMW E90 Sedan Simulation", irr.dimension2d_u32(1280, 720))
app.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0, 2.5, -6), irr.vector3df(0, 1.5, 0))
app.SetChaseCameraMode(True)
app.SetChaseCameraDistance(6)
app.SetChaseCameraVerticalOffset(1.5)


light = irr.ILightSceneNode()
light.setType(irr.ELT_DIRECTIONAL_LIGHT)
light.setDirection(irr.vector3df(1, -1, -1))
light.setDiffuseColor(irr.SColor(255, 255, 255, 255))
app.GetSceneManager().addLightSceneNode(light)


app.SetStepManage(True)
app.SetTimestep(0.001)
app.SetTryRealtime(True)

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    
    
    driver.Update()
    
    
    time = vehicle.GetSystem().GetChTime()
    if time >= 0.0 and time <= 100.0:
        driver.ApplyInputs()
        vehicle.Advance(0.001)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Advance(0.001)
    
    app.EndScene()