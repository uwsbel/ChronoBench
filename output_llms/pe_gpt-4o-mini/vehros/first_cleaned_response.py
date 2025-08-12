import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import rospy  
from std_msgs.msg import Float64  


rospy.init_node('hmmwv_simulation', anonymous=True)


pub_velocity = rospy.Publisher('/vehicle/velocity', Float64, queue_size=10)
pub_position = rospy.Publisher('/vehicle/position', Float64, queue_size=10)


sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath('/path/to/chrono/data')  


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod.SMART)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType.NONE)  
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
hmmwv.Initialize()


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.7)  
patch_mat.SetRestitution(0.1)  
terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)  
terrain.Initialize()


driver = veh.HMMWV_Driver(hmmwv)
driver.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


time_step = 0.01  
sim_time = 0.0

while vis.Run():
    
    sys.DoStepDynamics(time_step)

    
    velocity = hmmwv.GetVehicleSpeed()  
    position = hmmwv.GetVehiclePos()  

    
    pub_velocity.publish(Float64(velocity))
    pub_position.publish(Float64(position.x))  

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sim_time += time_step