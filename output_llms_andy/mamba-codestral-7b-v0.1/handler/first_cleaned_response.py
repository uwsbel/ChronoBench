import pychrono
import pychrono.irrlicht
import pychrono.ros


system = pychrono.ChSystemSMC()


system.Set_G_acc(pychrono.ChVectorD(0, -9.81, 0))


floor = pychrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, False)
floor.SetPos(pychrono.ChVectorD(0, -0.1, 0))
system.Add(floor)


box = pychrono.ChBodyEasyBox(1, 1, 1, 1000, True, False)
box.SetPos(pychrono.ChVectorD(0, 1, 0))
system.Add(box)


ros_manager = pychrono.ros.ChRosManager()


custom_handler = pychrono.ros.ChRosHandlerCustom()


ros_manager.SetClockHandler(custom_handler)
ros_manager.SetBodyHandler(custom_handler)
ros_manager.SetTransformHandler(custom_handler)


ros_manager.RegisterHandler(custom_handler)


while system.GetChTime() < 10:
    system.DoStepDynamics(0.01)
    ros_manager.Run()

    
    custom_handler.PublishIntegerMessage("my_topic", system.GetChTime() * 100)