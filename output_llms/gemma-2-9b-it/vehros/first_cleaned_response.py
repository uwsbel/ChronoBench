import pychrono as chrono
import pychrono.postprocess as postprocess
import roslib
import rospy


chrono.SetChronoDataPath("./")  
system = chrono.ChSystemNSC()



vehicle = chrono.ChBodyEasyCylinder(1.5, 1.5, 2.0, 1000)  
vehicle.SetPos(chrono.ChVectorD(0, 0, 0.5)) 
vehicle.SetBodyFixed(True)


contact_method = 'contact_point'


engine_type = 'internal_combustion'


tire_model = 'pavement'



terrain = chrono.ChBodyEasyBox(10, 10, 0.1, 1000)
terrain.SetPos(chrono.ChVectorD(0, 0, 0))


terrain.SetFriction(0.8, 0.6)
terrain.SetRestitution(0.2)



driver = chrono.ChDriver()  


rospy.init_node('pychrono_sim')





dt = 0.01  

while not rospy.is_shutdown():
    
    

    
    

    
    system.DoStepDynamics(dt)