import chrono as chrono
import numpy as np


chrono.Initialize()


m_scene = chrono.Scene()


m_body = chrono.ChBodyEasyBox(1, 1, 1, 1000)
m_body.SetBodyFixed(True)
m_body.SetPosition(chrono.ChVector(0, 0, 0))
m_scene.Add(m_body)


m_sensor_manager = chrono.ChSensorManager()
m_sensor_manager.SetSensorType(chrono.ChSensorManager.SENSOR_TYPE_LIDAR)
m_sensor_manager.SetSensorName("LidarSensor")
m_body.AddSensor(m_sensor_manager)


m_sensor_manager.SetSensorRange(10)
m_sensor_manager.SetSensorAngle(0.1)
m_sensor_manager.SetSensorNoise(0.1)
m_sensor_manager.SetSensorVisualization(True)
m_sensor_manager.SetSensorSave(True)


m_lidar_sensor = m_sensor_manager.GetSensor()
m_lidar_sensor.SetPosition(chrono.ChVector(0, 0, 5))


m_lidar_body = chrono.ChBody()
m_lidar_body.SetMass(1)
m_lidar_body.SetInertia(1)
m_lidar_body.SetBodyFixed(False)
m_lidar_body.SetPosition(chrono.ChVector(0, 0, 5))
m_lidar_body.SetRotation(chrono.ChQuaternion(0, 0, 0, 1))
m_scene.Add(m_lidar_body)


m_constraint = chrono.ChConstraintSemiImplicitSpringDamper()
m_constraint.SetBodyA(m_lidar_body)
m_constraint.SetBodyB(m_body)
m_constraint.SetSemiImplicitSpringDamper(1, 1, 1)
m_scene.Add(m_constraint)


m_scene.SetTimestep(0.01)
m_scene.SetMaxStep(0.01)


while m_scene.GetT() < 10:
    m_scene.DoStep()
    print(m_lidar_sensor.GetBuffer())


chrono.Destroy()