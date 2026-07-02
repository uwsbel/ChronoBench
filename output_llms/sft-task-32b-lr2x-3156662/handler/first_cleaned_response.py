import pychrono as chrono
import pychrono_ros as chrono_ros
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class IntPublisherHandler(chrono_ros.ChROSCustomHandler):
    def __init__(self, topic):
        super().__init__(50)  
        self.topic = topic
        self._node = None
        self._publisher = None
        self._counter = 0

    def Initialize(self):
        self._node = self.GetNode()
        self._publisher = self._node.create_publisher(Int32, self.topic, 10)
        return True

    def Update(self):
        msg = Int32()
        msg.data = self._counter
        self._publisher.publish(msg)
        self._counter += 1
        return True


def main():
    
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    
    floor_mat = chrono.ChMaterialSurfaceNSC()
    floor = chrono.ChBodyEasyBox(10, 10, 0.1, floor_mat, 1000, True, True)
    floor.SetPos(chrono.ChVector3d(0, 0, -0.05))
    floor.SetFixed(True)
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(floor)

    
    box_mat = chrono.ChMaterialSurfaceNSC()
    box_mat.SetFriction(0.5)
    box_mat.SetRestitution(0.2)
    box = chrono.ChBodyEasyBox(1, 1, 1, box_mat, 1000, True, True)
    box.SetPos(chrono.ChVector3d(0, 0, 4))
    box.SetMass(1.0)
    box.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
    system.Add(box)

    
    ros_manager = chrono_ros.ChROSManager()
    ros_manager.RegisterHandler(chrono_ros.ChROSClockHandler())
    ros_manager.RegisterHandler(chrono_ros.ChROSBodyHandler(system, "/chrono/body_info"))
    ros_manager.RegisterHandler(chrono_ros.ChROSTransformHandler(system, "/chrono/transforms"))
    ros_manager.RegisterHandler(IntPublisherHandler("/custom_int_topic"))

    
    step_size = 1e-3
    real_time_factor = 1.0
    next_time = time.time() + real_time_factor * step_size

    
    rclpy.init()

    
    while True:
        current_time = system.GetChTime()
        print(f"Simulation time: {current_time:.4f} seconds")
        
        
        system.DoStepDynamics(step_size)
        
        
        ros_manager.Update()
        
        
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        next_time += real_time_factor * step_size

        
        if current_time >= 5.0:
            break

    rclpy.shutdown()


if __name__ == "__main__":
    main()