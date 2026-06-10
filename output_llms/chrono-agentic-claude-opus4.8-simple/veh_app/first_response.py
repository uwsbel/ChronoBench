import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # core data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data path

initLoc = chrono.ChVector3d(0, 0, 0.4)                               # Gator spawn
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                           # QUNIT (identity)

gator = veh.Gator()                                                  # self-owning Gator wrapper
gator.SetContactMethod(chrono.ChContactMethod_NSC)                   # rigid terrain -> NSC
gator.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
gator.SetChassisFixed(False)                                         # chassis free to move
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))          # initial pose
gator.SetBrakeType(veh.BrakeType_SHAFTS)                             # shafts-based brake
gator.SetTireType(veh.TireModelType_TMEASY)                          # TMeasy tires
gator.SetTireStepSize(1e-3)                                          # tire integration step
gator.SetInitFwdVel(0.0)                                             # start from rest
gator.Initialize()                                                   # build the vehicle

system = gator.GetSystem()                                           # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # collision system (contact scene)

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())                                 # mass banner
print("DRIVELINE TEMPLATE: ", gator.GetVehicle().GetDriveline().GetTemplateName())    # driveline template
print("TIRE TEMPLATE: ", gator.GetVehicle().GetTire(0, veh.LEFT).GetTemplateName())   # tire template

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis: mesh
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension: primitives
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering: primitives
gator.SetWheelVisualizationType(veh.VisualizationType_NONE)          # wheels: none
gator.SetTireVisualizationType(veh.VisualizationType_MESH)           # tires: mesh

terrain = veh.RigidTerrain(system)                                   # rigid terrain owned by system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material
patch_mat.SetFriction(0.9)                                           # terrain friction
patch_mat.SetRestitution(0.01)                                       # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50.0, 50.0)     # 50x50 patch at origin
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))                        # patch color
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)  # tiled texture
terrain.Initialize()                                                 # build terrain

driver = veh.ChDriver(gator.GetVehicle())                            # plain (non-interactive) driver
driver.Initialize()                                                  # initialize driver
driver.SetSteering(0.5)                                              # scripted steering
driver.SetThrottle(0.2)                                              # scripted throttle

manager = sens.ChSensorManager(gator.GetSystem())                    # sensor manager over the system
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(1, 1, 1), 500.0)          # scene point light

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1.45),                                  # third-person offset on chassis
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))       # slight downward tilt
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),                                          # mounted on the chassis body
    10,                                                              # update_rate (Hz)
    offset_pose,                                                     # offset pose on the chassis
    1280, 720,                                                       # image width, height
    1.408)                                                           # horizontal FOV (rad)
cam.SetName("Third Person POV")                                      # sensor name
cam.SetLag(0)                                                        # no lag
cam.SetCollectionWindow(0)                                           # zero exposure window
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Gator Camera"))    # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                           # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                        # save RGB frames
manager.AddSensor(cam)                                               # register the camera

step_size = 1e-3                                                     # physics step
end_time = 30.0                                                      # simulation duration

time = 0
while time < end_time:                                               # headless time-bounded loop
    time = system.GetChTime()                                        # simulation clock

    driver_inputs = driver.GetInputs()                              # current driver inputs

    driver.Synchronize(time)                                         # advance driver state
    terrain.Synchronize(time)                                        # advance terrain
    gator.Synchronize(time, driver_inputs, terrain)                 # feed inputs to vehicle

    driver.Advance(step_size)                                        # step driver
    terrain.Advance(step_size)                                       # step terrain
    gator.Advance(step_size)                                         # step vehicle (steps the system)

    manager.Update()                                                 # pump sensors once per step

    buffer = cam.GetMostRecentRGBA8Buffer()                          # read camera buffer
    if buffer.HasData():                                             # only after first sensor tick
        print('Buffer received. Resolution: {0}x{1}'.format(buffer.Width, buffer.Height))
