import time

import pychrono as chrono
import pychrono.ros as chros

import rclpy
from std_msgs.msg import Int32






if hasattr(chrono, "ChVector3d"):
    ChVector = chrono.ChVector3d
else:
    ChVector = chrono.ChVectorD


def set_gravity(system, gx, gy, gz):
    if hasattr(system, "SetGravitationalAcceleration"):
        system.SetGravitationalAcceleration(ChVector(gx, gy, gz))
    else:
        system.Set_G_acc(ChVector(gx, gy, gz))


def make_nsc_material():
    if hasattr(chrono, "ChContactMaterialNSC"):
        mat = chrono.ChContactMaterialNSC()
    else:
        mat = chrono.ChMaterialSurfaceNSC()

    mat.SetFriction(0.6)
    mat.SetRestitution(0.1)

    if hasattr(mat, "SetCompliance"):
        mat.SetCompliance(1e-9)

    if hasattr(mat, "SetDampingF"):
        mat.SetDampingF(0.2)

    return mat






class CustomIntPublisherHandler(chros.ChROSHandler):
    

    def __init__(self, update_rate_hz, topic_name):
        super().__init__(update_rate_hz)

        self.topic_name = topic_name
        self.counter = 0

        self.node = None
        self.publisher = None

    def Initialize(self, interface):
        
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node("chrono_custom_int_publisher")
        self.publisher = self.node.create_publisher(Int32, self.topic_name, 10)

        return True

    def Tick(self, time):
        
        msg = Int32()
        msg.data = self.counter

        self.publisher.publish(msg)

        self.node.get_logger().debug(
            f"Published Int32 data={msg.data} at Chrono time={time:.3f}"
        )

        self.counter += 1

        
        rclpy.spin_once(self.node, timeout_sec=0.0)






def main():
    
    
    

    system = chrono.ChSystemNSC()
    set_gravity(system, 0.0, 0.0, -9.81)

    
    try:
        system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass

    
    try:
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.002)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.002)
    except Exception:
        pass

    
    
    

    contact_material = make_nsc_material()

    
    
    

    floor = chrono.ChBodyEasyBox(
        10.0,     
        10.0,     
        0.2,      
        1000.0,   
        True,     
        True,     
        contact_material
    )

    floor.SetName("fixed_floor")
    floor.SetPos(ChVector(0.0, 0.0, -0.1))
    floor.SetFixed(True)

    system.Add(floor)

    
    
    

    box = chrono.ChBodyEasyBox(
        0.5,      
        0.5,      
        0.5,      
        500.0,    
        True,     
        True,     
        contact_material
    )

    box.SetName("movable_box")
    box.SetPos(ChVector(0.0, 0.0, 1.0))
    box.SetFixed(False)

    system.Add(box)

    
    
    

    ros_manager = chros.ChROSManager()

    
    clock_handler = chros.ChROSClockHandler()
    ros_manager.RegisterHandler(clock_handler)

    
    body_handler = chros.ChROSBodyHandler(
        50.0,                 
        box,
        "/chrono/box/state"
    )
    ros_manager.RegisterHandler(body_handler)

    
    tf_handler = chros.ChROSTFHandler(50.0)

    
    
    tf_handler.AddTransform(floor, "world", "fixed_floor")
    tf_handler.AddTransform(box, "world", "movable_box")

    ros_manager.RegisterHandler(tf_handler)

    
    custom_int_handler = CustomIntPublisherHandler(
        update_rate_hz=10.0,
        topic_name="/chrono/custom_int"
    )
    ros_manager.RegisterHandler(custom_int_handler)

    
    ros_manager.Initialize()

    
    
    

    step_size = 1.0e-3
    end_time = 20.0

    realtime_timer = chrono.ChRealtimeStepTimer()

    print("Starting Chrono + ROS simulation.")
    print("Publishing:")
    print("  /clock")
    print("  /chrono/box/state")
    print("  /tf")
    print("  /chrono/custom_int")
    print("Press Ctrl+C to stop.")

    try:
        while system.GetChTime() < end_time:
            time_now = system.GetChTime()

            
            system.DoStepDynamics(step_size)

            
            ros_ok = ros_manager.Update(time_now, step_size)

            if ros_ok is False:
                print("ROS manager requested shutdown.")
                break

            
            realtime_timer.Spin(step_size)

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")

    finally:
        if custom_int_handler.node is not None:
            custom_int_handler.node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("Simulation finished.")


if __name__ == "__main__":
    main()