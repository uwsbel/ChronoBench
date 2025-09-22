import pychrono as ch
    import pychrono.ros as chros
    import rclpy

    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -20))

    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)
    sys.AddMaterialProperty(phys_mat)

    
    floor_mat = ch.ChContactMaterialNSC()
    floor_mat.SetFriction(0.9)
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, floor_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)  
    floor.SetName("base_link")
    sys.Add(floor)

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 3))
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    box.SetName("box")
    sys.AddBody(box)

    
    handler = chros.ChROSHandler()
    handler.SetNode(ros_node)
    handler.SetTopic("my_topic")
    handler.SetQueueSize(2)
    handler.Initialize()

    
    manager = chros.ChROSPythonManager()
    manager.AddHandler(handler)
    manager.Initialize()

    
    time_step = 1e-3

    
    time = 0
    time_end = 30
    time_step = 1e-3
    while time < time_end:
        time += time_step
        sys.DoStepDynamics(time_step)

        
        manager.Update(time)

        
        time = manager.GetChTime()

    
    ros_node.spin()

import pychrono as ch
import pychrono.ros as chros
import rclpy


sys = ch.ChSystemNSC()
sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -20))


phys_mat = ch.ChContactMaterialNSC()
phys_mat.SetFriction(0.5)
sys.AddMaterialProperty(phys_mat)


floor_mat = ch.ChContactMaterialNSC()
floor_mat.SetFriction(0.9)
floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, floor_mat)
floor.SetPos(ch.ChVector3d(0, 0, -1))
floor.SetFixed(True)  
floor.SetName("base_link")
sys.Add(floor)


box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
box.SetPos(ch.ChVector3d(0, 0, 3))
box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
box.SetName("box")
sys.AddBody(box)


handler = chros.ChROSHandler()
handler.SetNode(ros_node)
handler.SetTopic("my_topic")
handler.SetQueueSize(2)
handler.Initialize()


manager = chros.ChROSPythonManager()
manager.AddHandler(handler)
manager.Initialize()


time_step = 1e-3


time = 0
time_end = 30
time_step = 1e-3
while time < time_end:
    time += time_step
    sys.DoStepDynamics(time_step)

    
    manager.Update(time)

    
    time = manager.GetChTime()


ros_node.spin()