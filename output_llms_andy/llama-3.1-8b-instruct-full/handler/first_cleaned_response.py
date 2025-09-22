import pychrono as chrono
import pychrono.ros as chros


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  


phys_mat = chrono.ChContactMaterialNSC()
phys_mat.SetFriction(0.5)  


floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
floor.SetPos(chrono.ChVector3d(0, 0, -1))  
floor.SetFixed(True)  
sys.Add(floor)  


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
box.SetPos(chrono.ChVector3d(0, 0, 5))  
box.SetRot(chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(1, 0, 0)))  
sys.Add(box)  


manager = chros.ChROSPythonManager()
manager.SetClockType(chros.ChROSClockType_REALTIME)  
manager.SetBodyToReport(box)  
manager.SetTransformUpdateRate(10)  
manager.Initialize()  


handler = chros.ChROSHandlerPython()
handler.SetTopic('my_topic')  
handler.SetQueueSize(10)  
handler.SetRate(10)  
handler.RegisterCallback(lambda: 42)  
manager.RegisterHandler(handler)  


time = 0
while time < 30:
    time = sys.GetChTime()  
    manager.Update(time)  
    sys.DoStepDynamics(1e-3)