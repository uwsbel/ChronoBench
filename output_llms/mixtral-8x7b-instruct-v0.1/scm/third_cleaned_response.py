import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random




num_boxes = 10
box_size = 1
min_box_pos = chrono.ChVector3d(-50, -50, -5)
max_box_pos = chrono.ChVector3d(50, 50, 5)

for i in range(num_boxes):
    box_pos = chrono.ChVector3d(
        random.uniform(min_box_pos.x, max_box_pos.x),
        random.uniform(min_box_pos.y, max_box_pos.y),
        random.uniform(min_box_pos.z, max_box_pos.z)
    )

    if not vehicle.GetSystem().IsBodyPresent(box_pos):
        box = chrono.ChBodyEasyBox(box_size, 1, 1, vehicle.GetSystem())
        box.SetPos(box_pos)
        box.SetCollide(True)
        box.SetBodyFixed(True)


smgr = irr.ChVisualSystemIrrlicht.GetSensorManager()
light_positions = [
    chrono.ChVector3d(-10, 20, 20),
    chrono.ChVector3d(10, -20, 20),
    chrono.ChVector3d(-20, 0, -20),
]

for pos in light_positions:
    light = irr.ChIrrLightSensor()
    light.SetPos(pos)
    light.SetRange(20)
    light.SetColor(chrono.ChColor(1, 1, 1))
    smgr.AddSensor(light)


camera_sensor = irr.ChVisualSensor()
camera_sensor.SetPos(trackPoint)
camera_sensor.SetMode(irr.ChVisualSensor.MODE_CAMERA)
camera_sensor.SetCameraType(irr.ChVisualSensor.CAMERA_TYPE_PERSPECTIVE)
camera_sensor.SetResolution(1280, 720)
camera_sensor.SetFieldOfView(60)
camera_sensor.SetNearPlane(0.1)
camera_sensor.SetFarPlane(1000)
camera_sensor.SetActive(True)


smgr.AddSensor(camera_sensor)


vis.AddCamera(camera_sensor)