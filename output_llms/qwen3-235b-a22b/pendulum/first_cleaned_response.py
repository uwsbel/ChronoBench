import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


pendulum = chrono.ChBodyEasyBox(0.1, 1.0, 0.1, 1000)  
pendulum.SetPos(chrono.ChVectorD(0, -0.5, 0))  
pendulum.SetRot(chrono.ChQuaternionD(chrono.ChAngleAxisD(chrono.CH_C_PI / 4, chrono.ChVectorD(0, 0, 1))))  
system.Add(pendulum)


joint = chrono.ChRevoluteJoint(
    ground, pendulum,
    chrono.ChVectorD(0, 0, 0),  
    chrono.ChVectorD(0, 0.5, 0)  
)
system.Add(joint)


application = chronoirr.ChIrrApp(system, 'PyChrono Pendulum Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(0, 0, 3), chronoirr.vector3df(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


step_size = 0.01
log_interval = 0.5
current_log_time = 0


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    system.DoStepDynamics(step_size)

    current_log_time += step_size
    if current_log_time >= log_interval:
        pos = pendulum.GetPos()
        ang_vel = pendulum.GetWvel_loc()
        print(f"Time: {system.GetChTime():.2f}s | Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) | Angular Velocity: ({ang_vel.x:.3f}, {ang_vel.y:.3f}, {ang_vel.z:.3f})")
        current_log_time = 0