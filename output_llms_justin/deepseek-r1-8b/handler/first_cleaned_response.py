import chrono as c
from chrono_ros_msgs.msg import Integer

def main():
    
    c.init()
    
    
    floor = c.create_ground("floor", thickness=10, young_modulus=1e9, poisson_ratio=0.2)
    box = c.create_box("box", mass=1, size=(1, 1, 1), position=(0, 0, 0.5))
    
    
    ros_manager = c.RosManager()
    
    
    ros_manager.clock = c.RosClock(1000)  
    ros_manager.body = lambda body: body
    ros_manager.transform = lambda body, link, transform= None: None
    
    
    def ros_handler(connection, message):
        while True:
            msg = Integer()
            msg.data = int(msg.data) + 1
            connection.send(msg)
            c.sleep(1/10)  
    
    
    ros_manager.register_handler("status", ros_handler)
    
    
    c.start()
    
    
    while True:
        c.step()
        if c.ros.send_once("status", 10):
            pass
    
    
    c.end()

if __name__ == "__main__":
    main()

import chrono as c
from chrono_ros_msgs.msg import Integer

def main():
    
    c.init()
    
    
    floor = c.create_ground("floor", thickness=10, young_modulus=1e9, poisson_ratio=0.2)
    box = c.create_box("box", mass=1, size=(1, 1, 1), position=(0, 0, 0.5))
    
    
    ros_manager = c.RosManager()
    
    
    ros_manager.clock = c.RosClock(1000)  
    ros_manager.body = lambda body: body
    ros_manager.transform = lambda body, link, transform= None: None
    
    
    def ros_handler(connection, message):
        while True:
            msg = Integer()
            msg.data = int(msg.data) + 1
            connection.send(msg)
            c.sleep(1/10)  
    
    
    ros_manager.register_handler("status", ros_handler)
    
    
    c.start()
    
    
    while True:
        c.step()
        if c.ros.send_once("status", 10):
            pass
    
    
    c.end()

if __name__ == "__main__":
    main()