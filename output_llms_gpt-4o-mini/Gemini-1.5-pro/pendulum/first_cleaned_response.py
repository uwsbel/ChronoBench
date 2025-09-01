import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr






system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))






ground = chrono.ChBodyEasyBox(10, 0.2, 10, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)


pendulum_bob = chrono.ChBodyEasySphere(0.2, 1, True, True)
pendulum_bob.SetPos(chrono.ChVectorD(0, -1, 0))
system.Add(pendulum_bob)






revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(ground, pendulum_bob, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.Add(revolute_joint)






application = chronoirr.ChIrrApp(system, "Simple Pendulum Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(2, 2, 2))






data_logger = chrono.ChDataLoggerCSV("pendulum_data.csv")
data_logger.AddValue("time", lambda: system.GetChTime())
data_logger.AddValue("position_x", lambda: pendulum_bob.GetPos().x)
data_logger.AddValue("position_y", lambda: pendulum_bob.GetPos().y)
data_logger.AddValue("velocity_x", lambda: pendulum_bob.GetPos_dt().x)
data_logger.AddValue("velocity_y", lambda: pendulum_bob.GetPos_dt().y)
system.SetChDataLogger(data_logger)






time_step = 1e-3


simulation_duration = 10


application.AssetBindAll()
application.AssetUpdateAll()
application.SetTimestep(time_step)

while (application.GetDevice().run()) and (system.GetChTime() < simulation_duration):
    system.DoStepDynamics(time_step)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()


data_logger.Close()