import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

print(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, -5, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simulation Window')
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.SetInitialDamping(0.0)
gator.SetCompliance(0.01)
gator.SetFixed(True)
gator.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.SetCollisionModel(chrono.ChCollisionModel.Type_BULLET)
gator.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)



chassis = chrono.ChBody()
chassis.SetPos(chrono.ChVector3d(0, 0, 0.5))
chassis.SetFixed(True)
chassis.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))


suspension = chrono.ChBody()
suspension.SetPos(chrono.ChVector3d(0, 0, 0.5))
suspension.SetFixed(True)
suspension.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))


steering = chrono.ChBody()
steering.SetPos(chrono.ChVector3d(0, 0, 0.5))
steering.SetFixed(True)
steering.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))


wheel = chrono.ChBody()
wheel.SetPos(chrono.ChVector3d(0, 0, 0.5))
wheel.SetFixed(True)
wheel.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))


camera = chrono.ChVisualShapeCamera()
camera.SetPosition(chrono.ChVector3d(0, 0, 0.5))
camera.SetOrientation(chrono.ChQuaterniond(0, 0, 0))
camera.SetVisibility(chrono.ChVisibility_NONE)
camera.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
camera.SetTexture(chrono.GetChronoDataFile("textures/green.png"))


manager.AddSensor(camera)


driver = veh.ChDriver(chassis)
driver.Initialize()


print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")


gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()


box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.SetFixed(True)
box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().AddBody(box)


cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y ,0.5, 1, 1000)
cylinder.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder.SetFixed(True)
cylinder.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().AddBody(cylinder)


driver = veh.ChDriver(chassis)
driver.Initialize()


realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
while time < end_time:
    time = gator.GetSystem().GetChTime()
    
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time, driver_inputs, terrain)
    gator.Synchronize(time, driver_inputs, terrain)

    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    
    realtime_timer.Spin(step_size)