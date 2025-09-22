import chrono
from chrono.irrlicht import *
from chrono.collision import *
from chrono.vehicle import *


chrono.SetChronoDataPath('path/to/your/chrono/data')
my_ ChronoEngine = chrono_engine.ChronoEngine()



floor = my_ ChronoEngine.AddBody(chrono.ChBody.Truss())
floor.SetPos(chrono.ChVector(0, 0, 0))
floor_shape = my_ ChronoEngine.AddShape(chrono.ChCollisionModel.Box, floor, 10, 0.1, 10)


crank = my_ ChronoEngine.AddBody(chrono.ChBody.Rigid())
crank.SetPos(chrono.ChVector(0, 0, 0))
crank_shape = my_ ChronoEngine.AddShape(chrono.ChCollisionModel.Cylinder, crank, 0.1, 1)


connecting_rod = my_ ChronoEngine.AddBody(chrono.ChBody.Rigid())
connecting_rod.SetPos(chrono.ChVector(0, 0, 0))
connecting_rod_shape = my_ ChronoEngine.AddShape(chrono.ChCollisionModel.Cylinder, connecting_rod, 0.1, 2)


piston = my_ ChronoEngine.AddBody(chrono.ChBody.Rigid())
piston.SetPos(chrono.ChVector(0, 0, 0))
piston_shape = my_ ChronoEngine.AddShape(chrono.ChCollisionModel.Sphere, piston, 0.1)



my_ ChronoEngine.SetGroundBody(floor)



crank_joint = my_ ChronoEngine.AddJoint(chrono.ChLinkLockRevolute())
crank_joint.SetBody1(floor)
crank_joint.SetBody2(crank)
crank_joint.SetAxis(chrono.ChVector(0, 1, 0))
crank_joint.SetPos(chrono.ChVector(0, 0, 0))


connecting_rod_joint = my_ ChronoEngine.AddJoint(chrono.ChLinkLockRevolute())
connecting_rod_joint.SetBody1(crank)
connecting_rod_joint.SetBody2(connecting_rod)
connecting_rod_joint.SetAxis(chrono.ChVector(0, 1, 0))
connecting_rod_joint.SetPos(chrono.ChVector(0, 1, 0))


piston_joint = my_ ChronoEngine.AddJoint(chrono.ChLinkLockSpherical())
piston_joint.SetBody1(connecting_rod)
piston_joint.SetBody2(piston)
piston_joint.SetPos(chrono.ChVector(0, 2, 0))


crank_motor = my_ ChronoEngine.AddJoint(chrono.ChLinkMotorRotation())
crank_motor.SetBody1(floor)
crank_motor.SetBody2(crank)
crank_motor.SetAxis(chrono.ChVector(0, 1, 0))
crank_motor.SetAngle(0)
crank_motor.SetAngVel(chrono.ChVectorD(0, 10, 0))


my_application = chrono_irrlicht.ChIrrApp(my_ ChronoEngine, 'Crank-Slider Mechanism')
my_application.AddTypicalSky()
my_application.AddTypicalLogo(chrono.GetChronoDataPath() + '/logo_irrlicht.png')
my_application.AddTypicalCamera(chrono.ChVectorD(5, 5, 5))


my_application.AssetBundleSearchPath = ['path/to/your/chrono/data']
my_application.Run()