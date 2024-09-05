import chrono
from chrono import ChVector, ChCoordsys, ChFrame
from chrono.irrlicht import IrrlichtChronoApplication
from chrono.collision import ChCollisionModel
from chrono.vehicle import ChVehicleModel, ChTerrain, ChTire, ChWheel, ChChassis, ChSteer, ChEngine, ChTransmission, ChDriveline, ChClutch
from chrono.vehicle.utils import *


chrono.SetChDir(chrono.GetChronoDataPath())
my_application = IrrlichtChronoApplication("Turtlebot Simulation")


system = chrono.ChSystemNSC()
system.Set_G_acc(ChVector(0, -9.81, 0))  


ground = ChTerrain()
system.Add(ground)


turtlebot = ChVehicleModel()
turtlebot.SetChassis(ChAssable())
chassis = turtlebot.GetChassis()
chassis.SetPos(ChVector(0, 0, 0))
chassis.SetRot(ChQuaternion(1, 0, 0, 0))
system.Add(chassis)


my_application.AddTypicalSky()
my_application.AddTypicalLogo()
my_application.AddTypicalCamera(chassis.GetPos())
my_application.SetTimestep(1./240)


for i in range(1000):
    
    if i % 10 == 0:
        turtlebot.GetMotor(0).SetSpeed(1.0)  
        turtlebot.GetMotor(1).SetSpeed(-1.0)  
    else:
        turtlebot.GetMotor(0).SetSpeed(0.0)
        turtlebot.GetMotor(1).SetSpeed(0.0)

    
    system.DoStepDynamics(my_application.GetTimestep())
    my_application.DrawAll()
    my_application.GetDevice().sleep(my_application.GetTimestep()*1000)


my_application.Run()