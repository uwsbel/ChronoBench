import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono_ros as ros
import rospy
import numpy as np

# Initialize PyChrono environment and core components
chrono.ChEngine.Initialize()

# Set up the simulation parameters
contact_method = chrono.ChContactMethod_NSC
engine_type = veh.GetEngineType(veh.EngineModelType_SHAFTS)
tire_model = veh.GetTireModelType(veh.TireModelType_TMEASY)

# Create the HMMWV vehicle
hmmwv = veh.HMMWV(vehicle_mass=3000, engine_mass=600, transmission_mass=100, left_steering_mass=10, right_steering_mass=10)
hmmwv.SetContactMethod(contact_method)
hmmwv.SetEngineType(engine_type)
hmmwv.SetTireModelType(tire_model)
hmmwv.SetTireStepSize(1e-3)
hmmwv.SetChassisFixed(False)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)))

# Create the terrain
terrain = veh.RigidTerrain(hmmwv.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.GetSystem().GetMaterialRegister().RegisterMaterial("patch", patch_mat)
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.75), chrono.ChQuaterniond(1, 0, 0, 0)), 200, 200, 0.2, 0.2)
patch.SetContactMethod(contact_method)
patch.SetTexture(chrono.GetChronoDataFile("textures/tile4.jpg"), 200, 200)

# Initialize the driver system
driver = veh.ChWheeledVehicleDriver(hmmwv.GetVehicle())
driver.SetSteeringType(veh.DriverSteeringType_PURE_PURSUIT)
driver.SetDesiredVehicleSpeed(10)
driver.SetDesiredHeading(0)

# Initialize ROS and register handlers
ros_manager = ros.ChronoROSManager()
ros_manager.Initialize()
ros_manager.RegisterHandler(ros.ChronoROSTimeHandler(ros_manager, hmmwv.GetSystem()))
ros_manager.RegisterHandler(ros.ChronoROSDriverInputsHandler(ros_manager, driver))
ros_manager.RegisterHandler(ros.ChronoROSChassisHandler_FloatPosVelAng(ros_manager, hmmwv.GetVehicle().GetChassisBody(), "chassis"))
ros_manager.RegisterHandler(ros.ChronoROSChassisHandler_FloatPosVelAng(ros_manager, hmmwv.GetVehicle().GetChassisBody(), "chassis_frame"))
ros_manager.RegisterHandler(ros.ChronoROSWheelHandler_FloatPosVelAng(ros_manager, hmmwv.GetVehicle().GetWheel(0), "wheel_0"))
ros_manager.RegisterHandler(ros.ChronoROSWheelHandler_FloatPosVelAng(ros_manager, hmmwv.GetVehicle().GetWheel(1), "wheel_1"))
ros_manager.RegisterHandler(ros.ChronoROSWheelHandler_FloatPosVelAng(ros_manager, hmmwv.GetVehicle().GetWheel(2), "wheel_2"))
ros_manager.RegisterHandler(ros.ChronoROSWheelHandler_FloatPosVelAng(ros_manager, hmmwv.GetVehicle().GetWheel(3), "wheel_3"))

# Simulation loop
while not rospy.is_shutdown():
    hmmwv.GetSystem().Synchronize(ros_manager.GetSystemTimeStep())
    driver.Synchronize(ros_manager.GetSystemTimeStep())
    terrain.Synchronize(ros_manager.GetSystemTimeStep())
    hmmwv.GetSystem().Advance(ros_manager.GetSystemTimeStep())
    driver.Advance(ros_manager.GetSystemTimeStep())
    terrain.Advance(ros_manager.GetSystemTimeStep())
    ros_manager.Synchronize(ros_manager.GetSystemTimeStep())
    ros_manager.Update()