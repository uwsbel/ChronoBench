import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.postprocess as postprocess
import pychrono.robot as robot
import rospy
import rospkg
import os


chrono.SetChronoDataPath(os.path.dirname(os.path.abspath(__file__)) + '/chrono_data/')


sys = chrono.ChSystemNSC()


sys.SetContactMethod(chrono.ChContactMethod_NSC)


vehicle = veh.HMMWV()


vehicle.SetEngineType(veh.HMMWV::EngineType::ENGINE_GM12V71)


vehicle.SetTireType(veh.HMMWV::TireType::TIRE_PIRELLI_SCANIA)


vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1.5))
vehicle.SetChassisVisualizationType(veh.HMMWV::VisualizationType::MESH)
vehicle.SetSuspensionVisualizationType(veh.HMMWV::VisualizationType::PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.HMMWV::VisualizationType::PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.HMMWV::VisualizationType::MESH)


sys.Add(vehicle)


terrain = veh.RigidTerrain(sys.GetContactMethod())


terrain.SetContactFriction(0.9)
terrain.SetContactRestitution(0.01)
terrain.SetPlane(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
terrain.Initialize(sys)


driver = veh.ChIrrGuiDriver(sys, irr. irrlicht.GUI_BIG)


rospy.init_node('pychrono_node')
ros_manager = robot.ChRosManager(sys)


ros_manager.RegisterClockSynchronizer()
ros_manager.RegisterVehicleSystem(vehicle)
ros_manager.RegisterDriverInputs(driver)


while rospy.ok():
    
    sys.DoStepDynamics(0.01)
    driver.BeginScene()
    driver.Render()
    driver.EndScene()

    
    ros_manager.Update()