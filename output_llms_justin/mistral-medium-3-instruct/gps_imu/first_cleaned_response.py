import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np
import time






step_size = 1e-3


render_step_size = 1.0 / 50  


imu_location = chrono.ChVectorD(0.5, 0, 1.0)
gps_location = chrono.ChVectorD(0.5, 0, 1.0)






vehicle = veh.ChVehicle()
vehicle.SetChassisFixed(False)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType.NONE)


terrain = veh.RigidTerrain(vehicle.GetSystem())


driver = veh.ChDriver()
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType.PRIMITIVES)
hmmwv.SetChassisVisualizationType(veh.ChassisVisualizationType.PRIMITIVES)
hmmwv.SetTireType(veh.TireModelType.TMEASY)
hmmwv.SetTireStepSize(step_size)

hmmwv.Initialize(chrono.ChCoordinateSystemD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT),
                chrono.ChCoordinateSystemD(chrono.ChVectorD(0, 0, -1)))

hmmwv.GetSystem().Add(vehicle.GetSystem())


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachVehicle(hmmwv.GetVehicle())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(8, 0.5, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.SetCameraMoveScale(0.002)






imu = veh.ChIMUSensor(hmmwv.GetVehicle().GetChassisBody(),
                     chrono.ChVectorD(imu_location),
                     chrono.ChQuaternionD(1, 0, 0, 0),
                     step_size)
hmmwv.GetVehicle().AddSensor(imu)


gps = veh.ChGPSSensor(hmmwv.GetVehicle().GetChassisBody(),
                     chrono.ChVectorD(gps_location),
                     chrono.ChQuaternionD(1, 0, 0, 0),
                     step_size)
hmmwv.GetVehicle().AddSensor(gps)






render_steps = int(np.ceil(render_step_size / step_size))


step_number = 0


output = []


while vis.Run():
    time_start = time.time()

    
    vehicle_state = hmmwv.GetVehicle().GetVehicleState()

    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time_start)

    
    hmmwv.Synchronize(time_start, driver_inputs, terrain)
    hmmwv.Advance(step_size)

    
    hmmwv.GetVehicle().UpdateSensors(step_size)

    
    imu_data = imu.GetData()
    gps_data = gps.GetData()

    
    if step_number % render_steps == 0:
        mass = hmmwv.GetVehicle().GetChassisBody().GetMass()
        print(f"Time: {time_start:.2f} s")
        print(f"Vehicle mass: {mass:.2f} kg")
        print(f"IMU data - Acceleration: {imu_data.m_acceleration}")
        print(f"IMU data - Angular velocity: {imu_data.m_ang_velocity}")
        print(f"GPS data - Position: {gps_data.m_position}")
        print("----------------------------------")

        
        output.append({
            'time': time_start,
            'mass': mass,
            'imu_accel': imu_data.m_acceleration,
            'imu_ang_vel': imu_data.m_ang_velocity,
            'gps_pos': gps_data.m_position
        })

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    step_number += 1