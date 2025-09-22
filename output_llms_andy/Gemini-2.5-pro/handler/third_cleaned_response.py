import pychrono as ch
import pychrono.irrlicht as irr 
import pychrono.ros as chros


import rclpy.publisher 
from std_msgs.msg import Int64



try:
    
    
    
    chrono_data_path = ch.GetChronoDataPath()
    if chrono_data_path: 
        ch.SetChronoDataPath(chrono_data_path)
    else: 
        print("Warning: CHRONO_DATA_DIR environment variable not found or Chrono data path not set.")
        print("Attempting to use a default relative path './chrono_data/' for data.")
        print("If textures/logo are not found, please set CHRONO_DATA_DIR or use ch.SetChronoDataPath().")
        
        
except Exception as e:
    print(f"Warning: Could not decisively set Chrono data path. Textures might not load. {e}")



class MyCustomHandler(chros.ChROSHandler):
    

    
    def __init__(self, update_rate: float, topic: str):
        super().__init__(update_rate)  

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None 
        self.ticker = 0  

    
    def Initialize(self, interface: chros.ChROSPythonManager) -> bool:
        
        print(f"MyCustomHandler: Creating publisher for topic {self.topic} ...")
        
        
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 10) 
        return True  

    def Tick(self, time: float):
        
        
        
        msg = Int64()  
        msg.data = self.ticker  
        self.publisher.publish(msg)  
        self.ticker += 1  

def main():
    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))  

    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  

    
    
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -0.5))
    floor.SetFixed(True)  
    floor.SetName("base_link")  
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 2))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    sys.Add(box)  

    
    try:
        floor_texture_path = ch.GetChronoDataFile('textures/concrete.jpg')
        box_texture_path = ch.GetChronoDataFile('textures/bluewhite.png')
        
        floor_vis_shape = floor.GetVisualShape(0) 
        if floor_vis_shape:
             floor_vis_shape.SetTexture(floor_texture_path)

        box_vis_shape = box.GetVisualShape(0)
        if box_vis_shape:
            box_vis_shape.SetTexture(box_texture_path)
            
    except Exception as e:
        print(f"Error setting textures: {e}. Make sure Chrono data path is correct and textures exist.")

    
    ros_manager = chros.ChROSPythonManager()
    
    
    publish_rate = 10.0  

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box_state"))
    
    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    
    tf_handler.AddTransform(floor, floor.GetName()) 
    
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    
    custom_handler = MyCustomHandler(publish_rate, "~/my_topic_data")
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    myapplication = irr.ChVisualSystemIrrlicht()
    myapplication.AttachSystem(sys)
    myapplication.SetWindowTitle('PyChrono ROS Simulation with Irrlicht')
    myapplication.SetWindowSize(1024, 768)
    myapplication.Initialize()
    try:
        myapplication.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    except Exception as e:
        print(f"Could not load logo: {e}")
    myapplication.AddSkyBox()
    myapplication.AddTypicalLights()
    myapplication.AddCamera(ch.ChVector3d(3, -4, 3), ch.ChVector3d(0, 0, 0.5)) 
    myapplication.BindAll() 


    
    time = 0.0
    time_step = 1e-3  
    time_end = 30.0  

    
    step_number = 0
    render_step_size = time_step 
    
    render_steps = 40 

    realtime_timer = ch.ChRealtimeStepTimer()
    
    print("Starting simulation loop...")
    while myapplication.GetDevice().run() and time < time_end:
        
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        
        if not ros
print("error happened with only start ```python")