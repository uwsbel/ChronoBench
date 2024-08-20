import pychrono as ch
import pychrono.ros as chros
import irrlicht
import irrlicht.core as core
import irrlicht.video as video
import irrlicht.gui as gui
import irrlicht.scene as scene
import irrlicht.image as image


class MyCustomHandler(chros.ChROSPythonHandler):
    

    def __init__(self, topic, publish_rate=10):
        super().__init__(1)  
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0
        self.publish_rate = publish_rate

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        print(f"Creating publisher for topic {self.topic} ...")
        
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, self.publish_rate)
        return True  

    def Tick(self, time: float):
        print(f"Publishing {self.ticker} ...")
        msg = Int64()  
        msg.data = self.ticker  
        self.publisher.publish(msg)  
        self.ticker += 1  


irrlicht_window_width = 800
irrlicht_window_height = 600
irrlicht_window_title = "PyChrono Simulation"


engine = core.IrrlichtEngine()


window = engine.createWindow(irrlicht_window_width, irrlicht_window_height, irrlicht_window_title)


camera = window.getCamera()
camera.setPosition(ch.ChVector3d(0, 0, 10))  
camera.setLookAt(ch.ChVector3d(0, 0, 0))  


light = scene.Light()
light.setPosition(ch.ChVector3d(0, 10, 0))  
light.setColor(ch.ChColor(1, 1, 1, 1))  


scene = scene.Scene()
scene.addLight(light)


floor_texture = image.loadTexture("floor.png")
box_texture = image.loadTexture("box.png")


floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, ch.ChContactMaterialNSC())
floor.SetPos(ch.ChVector3d(0, 0, -1))  
floor.SetFixed(True)  
floor.SetName("base_link")  
scene.addBody(floor)

box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, ch.ChContactMaterialNSC())
box.SetPos(ch.ChVector3d(0, 0, 5))  
box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
box.SetName("box")  
scene.addBody(box)


ros_manager = chros.ChROSPythonManager()


ros_manager.RegisterHandler(chros.ChROSClockHandler())


ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))


tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)


custom_handler = MyCustomHandler("~/my_topic", publish_rate=10)
ros_manager.RegisterPythonHandler(custom_handler)


ros_manager.Initialize()


time = 0
time_step = 1e-3  
time_end = 30  
realtime_timer = ch.ChRealtimeStepTimer()

while time < time_end:
    sys.DoStepDynamics(time_step)  
    time = sys.GetChTime()  

    if not ros_manager.Update(time, time_step):  
        break  

    realtime_timer.Spin(time_step)  


while engine.isRunning():
    engine.run()


engine.shutdown()