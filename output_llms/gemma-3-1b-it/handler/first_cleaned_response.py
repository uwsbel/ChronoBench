import time
import pychrono
import rospy
from pychrono.system import Chrono
from pychrono.system.system import System




system = System()
system.name = "BoxSimulation"
system.description = "A simple simulation of a box moving on a floor with gravity."
system.update_time_step = 1.0  
system.update_time_step_interval = 1.0 
system.update_time_step_interval_interval = 1.0 
system.gravity = 9.81  
system.material_density = 0.8 
system.box_mass = 10.0 
system.box_size = 0.2 




box = rospy.Action("box_move",
                    paramet=rospy.Parameter("x", type=float, default=0.0),
                    paramet=rospy.Parameter("y", type=float, default=0.0),
                    paramet=rospy.Parameter("z", type=float, default=0.0),
                    paramet=rospy.Parameter("mass", type=float, default=10.0),
                    paramet=rospy.Parameter("size", type=float, default=0.2),
                    reconnect=True)


floor = rospy.Action("floor_move",
                    paramet=rospy.Parameter("x", type=float, default=0.0),
                    paramet=rospy.Parameter("y", type=float, default=0.0),
                    paramet=rospy.Parameter("z", type=float, default=0.0),
                    paramet=rospy.Parameter("mass", type=float, default=10.0),
                    paramet=rospy.Parameter("size", type=float, default=0.2),
                    reconnect=True)




handler = rospy.Action("publish_message",
                         paramet=rospy.ActionParameters("message_topic", "box_movement_message"),
                         reconnect=True)



def run_simulation():
    rospy.init_node('box_simulation')
    system.subscribe(handler)
    system.wait_for_condition(system.gravity)  

    try:
        while True:
            
            system.update_time_step()
            system.move_box(x=system.box.x, y=system.box.y, z=system.box.z)
            system.update_time_step()
            
            floor.update_time_step()
            floor.move_box(x=floor.x, y=floor.y, z=floor.z)
            system.update_time_step()

            
            handler.publish(system.message)

            
            time.sleep(0.1)  

    except rospy.EventRobotException:
        print("Robot exception occurred.  Stopping simulation.")
    except rospy.ROSInterruptException:
        print("Simulation interrupted.")
    finally:
        system.shutdown()


if __name__ == '__main__':
    run_simulation()