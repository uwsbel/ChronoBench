import pychrono.core as chrono
import pychrono.engine as chronoengine
import pychrono.vehicle as chrono_vehicle
import pychrono.irrlicht as chronoirr
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()
chrono.ChCollisionSystem.SetDefaultImpulseThreshold(1e-3)


hmmwv = chrono_vehicle.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod_MULTICORE_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetEngineType(chrono_vehicle.EngineType_INTERNAL_COMBUSTION)
hmmwv.SetTireModel(chrono_vehicle.TireModel_RIGID_RING)
hmmwv.SetTireFriction(chrono_vehicle.TireFrictionModel_SLIPWEAR)
hmmwv.SetTireSlip(chrono_vehicle.TireSlipModel_SLIPWEAR)
hmmwv.SetTireWear(chrono_vehicle.TireWearModel_SLIPWEAR)


mysystem.Add(hmmwv.GetChassisBody())
mysystem.Add(hmmwv.GetChassisBodyAuxRef())


terrain = chrono.ChBodyEasyBox(mysystem, 10, 10, 1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterialSurface().SetFriction(0.9)
terrain.GetMaterialSurface().SetRestitution(0.1)


mysystem.Add(terrain)


driver = chrono_vehicle.ChIrrGuiDriver()
driver.SetVehicle(hmmwv)
driver.SetSteeringMode(chrono_vehicle.SteeringMode_ACKERMANN)


rospy.init_node('pychrono_simulation')


ros_manager = chrono.ChROSManager()


ros_manager.RegisterHandler(chrono.ChROSHandlerClockSync())
ros_manager.RegisterHandler(chrono.ChROSHandlerDriverInputs(driver))
ros_manager.RegisterHandler(chrono.ChROSHandlerVehicleState(hmmwv))


vehicle_state_pub = rospy.Publisher('vehicle_state', Float64MultiArray, 10)


while not rospy.is_shutdown():
    
    ros_manager.Synchronize()

    
    mysystem.DoStepDynamics(0.01)

    
    ros_manager.Update()

    
    vehicle_state = Float64MultiArray()
    vehicle_state.data = [hmmwv.GetChassisBody().GetPos().x, hmmwv.GetChassisBody().GetPos().y, hmmwv.GetChassisBody().GetPos().z,
                          hmmwv.GetChassisBody().GetRot().Q_to_Euler123().x, hmmwv.GetChassisBody().GetRot().Q_to_Euler123().y, hmmwv.GetChassisBody().GetRot().Q_to_Euler123().z,
                          hmmwv.GetSpeed(), hmmwv.GetSteeringAngle(), hmmwv.GetThrottle()]
    vehicle_state_pub.publish(vehicle_state)

    
    driver.Render()