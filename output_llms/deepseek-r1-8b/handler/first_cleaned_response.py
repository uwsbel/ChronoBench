import pychrono as pyc
import chrono as c
import pychrono.ros as ros


c.setgravity((0, 0, -9.81))  
c.defaultmaterial = c.MaterialProperties(c.MATERIAL_TYPE_HARD, 1e9, 0.2)  


floor_body = c.Body("floor", mass=0, position=(0, 0, 0), size=(10, 10, 10))
floor_body.setfixed()
floor_body.setmaterial(c.defaultmaterial)
floor_body.setrotationfixed(True)


box_body = c.Body("box", mass=1, position=(0, 1, 0), size=(5, 1, 5))
box_body.setposition((0, 1, 0))
box_body.setmaterial(c.defaultmaterial)


class ROSHandler(ros.Handler):
    def __init__(self):
        super().__init__()
        self.subscribed_topic = None
        self.status_message = 0

    def subscribe(self, topic, callback=None):
        self.subscribed_topic = topic
        self.callback = callback

    def handle(self):
        if self.subscribed_topic:
            data = self.status_message
            self.publish(self.subscribed_topic, data)

def handler_callback(msg):
    print(f"Received message: {msg.data}")


ros_manager = ros.RosManager()
ros_manager.add_clock_handler(100)  
ros_manager.add_body_handler(floor_body, 50)  
ros_manager.add_body_handler(box_body, 50)  
ros_manager.add_transform_handler(box_body)  


ros_manager.register_handler(ROSHandler())
ros_manager.start()


t = 0
while True:
    c.simulate(t=1e-4)  
    t += 1e-4
    ros_manager.update()  
    ros_handler = ROSHandler()
    ros_handler.status_message = int(t * 1000)  
    ros_manager.publish(ros_handler)  

    if t > 10:  
        break


c.close()
ros_manager.close()