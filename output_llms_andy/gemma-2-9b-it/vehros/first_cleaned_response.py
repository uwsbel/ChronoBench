import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono_ros  


chrono.SetChronoData(chrono.ChSystemNSC.GetDefault())  
system = chrono.ChSystemNSC()



vehicle = chrono.ChBodyEasy.CreatePrimitive(
    chrono.ChBodyEasy.Shape.BOX, 
    0.5, 0.5, 0.5, 1000.0, 
    chrono.ChVectorD(0, 0, 0)  
)
system.Add(vehicle)














ros_manager = pychrono_ros.RosManager(system)






dt = 0.01  

while True:
    
    ros_manager.step()

    
    
    

    

    system.DoStepDynamics(dt)
    system.Render()