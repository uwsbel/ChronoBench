import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np
import rospy
from rospkg import RosPack


chrono.SetChronoDataPath('/path/to/chrono/data')


contact_method = chrono.ChContactMethod_NSC
vehicle_model = "hmmwv"
engine_model = "simple"
tire_model = "rigidtire"


initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


terrain_mat = chrono.ChMaterialSurfaceNSC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(terrain_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


driver = veh.ChIrrGuiDriver(vehicle, "HMMWV Demo")
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


rospy.init_node('hmmwv_simulator')
ros_manager = ROSManager()


ros_manager.register_clock_handler()
ros_manager.register_driver_input_handler(driver)
ros_manager.register_vehicle_state_handler(vehicle)


while not rospy.is_shutdown():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(0.01)
    terrain.Advance(0.01)
    driver.Advance(0.01)
    ros_manager.publish_data(vehicle, driver_inputs)
    rospy.sleep(0.01)

class ROSManager:
    def __init__(self):
        
        self.clock_pub = rospy.Publisher('/clock', Clock, 10)
        self.driver_input_pub = rospy.Publisher('/driver_inputs', DriverInputs, 10)
        self.vehicle_state_pub = rospy.Publisher('/vehicle_state', VehicleState, 10)

    def register_clock_handler(self):
        
        pass

    def register_driver_input_handler(self, driver):
        
        pass

    def register_vehicle_state_handler(self, vehicle):
        
        pass

    def publish_data(self, vehicle, driver_inputs):
        
        clock_msg = Clock()
        clock_msg.clock = rospy.Time.now()
        self.clock_pub.publish(clock_msg)

        driver_input_msg = DriverInputs()
        
        self.driver_input_pub.publish(driver_input_msg)

        vehicle_state_msg = VehicleState()
        
        self.vehicle_state_pub.publish(vehicle_state_msg)