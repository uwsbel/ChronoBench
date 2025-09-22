import pychrono as pyc
import chrono as c
import pychrono.ros as ros


c.system.setGravity((0, -9.81, 0))  


floor = c.Box(mass=1000,  
             material=c.Material.STEEL,  
             size=(10, 0.01, 10),  
             position=(0, 0, 0))  


box = c.Box(mass=1,  
           material=c.Material.ALUMINUM,  
           size=(0.5, 0.5, 0.5),  
           position=(0.5, 0.5, 0.5))  


class ROSHandler(pyc.ros.RosHandler):
    def __init__(self):
        super().__init__()
        
    def setup(self):
        self.pub = self.create_publisher("object_position", 1)  
    
    def update(self, t):
        
        pos = self.body.get_position()
        
        position = [int(pos[0]*1000), int(pos[1]*1000), int(pos[2]*1000)]
        self.pub.publish(position)


ros_manager = c.ros.RosManager()
ros_manager.setClockHandler()  
ros_manager.setBodyHandler(box)  



ros_manager.registerHandler(ROSHandler(), priority=1)  


t = 0.0
while True:
    
    c.system.advance(0.01)
    t += 0.01
    
    
    ros_manager.update()
    
    
    print(f"Simulation time: {t:.3f}")
    print("Movable box position:", box.get_position())
    
    
    if t >= 10:
        c.system.stopSimulation()
        break


print("\nSimulation completed!")