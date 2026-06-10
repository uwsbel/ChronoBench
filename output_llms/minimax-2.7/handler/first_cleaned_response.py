import rospy
import std_msgs.msg                     
import pychrono.core as chrono          
import pychrono.ros as chronoros       




class CustomIntHandler(chronoros.ChROSBaseHandler):
    
    def __init__(self, topic: str):
        super().__init__()
        
        self.pub = rospy.Publisher(topic, std_msgs.msg.Int64, queue_size=10)

    def broadcast(self, time: float):
        
        msg = std_msgs.msg.Int64()
        msg.data = int(time * 1000)          
        self.pub.publish(msg)





def make_box(name: str,
             width: float, height: float, depth: float,
             density: float,
             material: chrono.ChMaterialSurface,
             fixed: bool,
             pos: chrono.ChVectorD) -> chrono.ChBody:
    
    body = chrono.ChBodyEasyBox(width, height, depth,
                                density, material)
    body.SetName(name)
    body.SetPos(pos)
    body.SetFixed(fixed)
    return body





def main():
    
    
    
    rospy.init_node("chrono_ros_example", anonymous=True)
    rospy.loginfo("Chrono‑ROS example node started")

    
    
    
    system = chrono.ChSystemNSC()               
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))   

    
    step_size = 0.001            
    end_time = 20.0              

    
    
    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.3)
    material.SetRestitution(0.1)

    
    
    
    floor = make_box(
        name="floor",
        width=10.0, height=0.2, depth=10.0,
        density=1000.0,
        material=material,
        fixed=True,
        pos=chrono.ChVectorD(0.0, -0.1, 0.0)   
    )
    system.Add(floor)

    
    
    
    box = make_box(
        name="box",
        width=0.5, height=0.5, depth=0.5,
        density=100.0,
        material=material,
        fixed=False,
        pos=chrono.ChVectorD(0.0, 2.0, 0.0)   
    )
    
    box.SetVel(chrono.ChVectorD(0.1, 0.0, 0.0))
    system.Add(box)

    
    
    
    manager = chronoros.ChROSManager()

    
    clock_hdl = chronoros.ChROSClockHandler()
    manager.add_handler(clock_hdl)

    
    body_hdl = chronoros.ChROSSBodyHandler(box, "/box/state")
    manager.add_handler(body_hdl)

    
    tf_hdl = chronoros.ChROSTransformHandler()
    tf_hdl.set_parent_frame("world")
    tf_hdl.add_body("box", box)
    manager.add_handler(tf_hdl)

    
    int_hdl = CustomIntHandler(topic="/chrono/sim_time_ms")
    manager.add_handler(int_hdl)

    
    manager.start()

    
    
    
    sleeper = chrono.ChRealtimeSleeper()

    
    
    
    sim_time = 0.0
    while not rospy.is_shutdown():
        
        if end_time > 0.0 and sim_time >= end_time:
            rospy.loginfo("Reached end time – stopping simulation.")
            break

        
        system.DoStepDynamics(step_size)
        sim_time += step_size

        
        manager.update(sim_time)

        
        sleeper.sleep(step_size)

    rospy.loginfo("Simulation loop terminated – shutting down.")





if __name__ == "__main__":
    main()