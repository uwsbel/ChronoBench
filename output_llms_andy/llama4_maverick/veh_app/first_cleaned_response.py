import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math


print('Copyright (c) 2023 Project Chrono')


vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(-5, -2, 0.6), chrono.Q_from_AngZ(0)))
vehicle.SetInitFwdVel(5)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(-math.pi / 2)), chrono.ChVector3d(20, 20), 0.01)
terrain.Initialize()


driver = veh.ChDriver(vehicle, "myfile.json")  


manager = sens.ChSensorManager(vehicle.GetSystem())
manager.scene.Add(terrain)


offset_pose = chrono.ChFrameD(chrono.ChVector3d(-5, 0, 2), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0)))
camera = sens.ChCameraSensor(vehicle.GetChassis().GetBody(), 30, offset_pose, 1280, 720, chrono.Q_from_AngZ(chrono.CH_C_PI_2))
camera.SetName("Camera")
manager.AddSensor(camera)


time_step = 1e-3
time_end = 1000
realtime_timer = chrono.ChRealtimeStepTimer()
while vehicle.GetSystem().GetChTime() < time_end:
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(time_step)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    manager.Update()
    manager.Render()
    realtime_timer.Spin(time_step)