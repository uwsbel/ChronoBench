import pychrono as chrono
import rclpy
from rclpy.node import Node
from chrono_ros_msgs.srv import SetParameters
from chrono_ros_msgs.msg import SimulationParameters
from chrono_ros_msgs.srv import GetParameters
from chrono_ros_msgs.msg import SimulationStatus
from chrono_ros_msgs.msg import SensorData
from chrono_ros_msgs.msg import SensorStatus


rclpy.init(args=['--ros-args', '--log-level=info'])
node = Node('pychrono_simulation_node')


sim = chrono.ChSystemNSC()


ground_body = chrono.createBody(
    name='ground_body',
    mass=1e4,
    rest_position=chrono.ChVectorD(0, 0, 0),
    rest_velocity=chrono.ChVectorD(0, 0, 0),
    density=1000,
    friction_coefficient=0.3,
    rest_rotation=chrono.ChQuaternionD(0, 0, 0, 1)
)
sim.Add(ground_body)


ground_mesh = chrono.Mesh('ground_mesh.obj')
sim.Add(ground_mesh)


ground_body.AddApplyBodyForce(
    chrono.ChVectorD(0, 0, -9.81 * 10),  
    chrono.ChFrameX(chrono.ChVectorD(0, 0, 10),
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(0, 0, -1),
    chrono.ChForceGroup.kAir
)


sensor_manager = chrono.SensorManager()


camera_topic = 'camera_sensor/image'
lidar_topic = 'lidar_sensor/data'
gps_topic = 'gps_sensor/data'
accelerometer_topic = 'accelerometer_sensor/data'
gyroscope_topic = 'gyroscope_sensor/data'
magnetometer_topic = 'magnetometer_sensor/data'


camera_publisher = node.create_publisher(chrono_ros_msgs.msg.SensorData, camera_topic, 10)
lidar_publisher = node.create_publisher(chrono_ros_msgs.msg.SensorData, lidar_topic, 10)
accelerometer_publisher = node.create_publisher(chrono_ros_msgs.msg.SensorData, accelerometer_topic, 10)
gyroscope_publisher = node.create_publisher(chrono_ros_msgs.msg.SensorData, gyroscope_topic, 10)
magnetometer_publisher = node.create_publisher(chrono_ros_msgs.msg.SensorData, magnetometer_topic, 10)


def update_sensors_and_publish(sim):
    
    camera_data = chrono_ros_msgs.msg.SensorData(image_data="...")
    lidar_data = chrono_ros_msgs.msg.SensorData(data_points=[...])
    accelerometer_data = chrono_ros_msgs.msg.SensorData(acceleration=chrono.ChVectorD(0, 0, -9.81))
    gyroscope_data = chrono_ros_msgs.msg.SensorData(angular_velocity=chrono.ChVectorD(0, 0, 0))
    magnetometer_data = chrono_ros_msgs.msg.SensorData(magnetic_field=chrono.ChVectorD(0, 0, 0))

    
    camera_publisher.publish(camera_data)
    lidar_publisher.publish(lidar_data)
    accelerometer_publisher.publish(accelerometer_data)
    gyroscope_publisher.publish(gyroscope_data)
    magnetometer_publisher.publish(magnetometer_data)


def simulation_loop(sim):
    while rclpy.ok():
        
        sim.SetParam('time_step', 1e-3)
        sim.Step(1e-3, 1e-3)

        
        update_sensors_and_publish(sim)

        
        status = sim.GetStatus()
        print(f"Simulation status: {status}")

        
        rclpy.sleep(1e-3)


srv_set_params = node.service_create(SetParameters, 'set_parameters')
def set_parameters_callback(request, response):
    sim.SetParam('time_step', request.time_step)
    return response


srv_get_params = node.service_create(GetParameters, 'get_parameters')
def get_parameters_callback(request, response):
    response.simulation_parameters = sim.GetParameters()
    return response


def main():
    try:
        
        simulation_loop(sim)
    except Exception as e:
        node.get_logger().error(f"Simulation error: {str(e)}
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()