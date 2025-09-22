import pychrono as ch
import pychrono.ros as chros
import pyrr
import Irrlicht

Irr.IrrlichtDevice.setWindowCaption(u"PyChrono Simulation")
Irr.IrrlichtDevice.setResizable(True)
Irr.IrrlichtDevice.setEventReceiver(sys)

driver = Irr.createDevice(Irr.EDT_OPENGL, Irr.size(800, 600))
smgr = driver.getSceneManager()


camera = smgr.addCameraSceneNodeFPS()
camera.setPosition(ch.ChVectorD(10, 10, 10))
camera.setTarget(ch.ChVectorD(0, 0, 0))


ambient_light = smgr.addAmbientLight()
ambient_light.setColor(Irr.SColor(255, 100, 100, 100))

point_light = smgr.addLightSceneNode(0, ch.ChVectorD(0, 10, 0))
point_light.setPosition(ch.ChVectorD(0, 10, 0))
point_light.setDiffuseColor(Irr.SColor(255, 255, 255, 255))


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(10)  

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Creating publisher for topic {self.topic} ...")
        
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 10)
        return True  

    def Tick(self, time: float):
        
        print(f"Publishing {self.ticker} ...")
        msg = Int64()  
        msg.data = self.ticker  
        self.publisher.publish(msg)  
        self.ticker += 1  

publish_rate = 10  
step_number = 0  
render_step_size = 5  
render_steps = 10  

def main():
    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))  

    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  

    
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))  
    floor.SetFixed(True)  
    floor.SetName("base_link")  
    floor.SetTexture(driver, u"floor.jpg")  
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    box.SetTexture(driver, u"box.jpg")  
    sys.Add(box)  

    
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))

    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        step_number += 1  

        if step_number % render_steps == 0:  
            
            smgr.clear()

            
            floor_node = ch.ChIrrMeshVisualization.AddVisualization(sys, floor, smgr)
            box_node = ch.ChIrrMeshVisualization.AddVisualization(sys, box, smgr)

            
            camera_position = ch.ChVectorD(floor.GetPos().x, 5, floor.GetPos().z + 5)
            camera.setPosition(camera_position)
            camera.setTarget(floor.GetPos())

            
            driver.beginScene(True, True, Irr.SColor(255, 128, 128, 128))
            smgr.drawAll()
            driver.endScene()

        if not ros_manager.Update(time, time_step):  
            break  

        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()