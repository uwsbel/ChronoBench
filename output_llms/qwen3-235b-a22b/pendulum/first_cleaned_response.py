import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1.0)  
pendulum.SetInertiaXX(chrono.ChVectorD(0.333, 0.333, 0.333))  


theta = math.radians(45)
pendulum.SetPos(chrono.ChVectorD(0, -0.5, 0))  
pendulum.SetRot(chrono.ChQuaternionD(math.cos(theta/2), 0, 0, math.sin(theta/2)))  

sys.Add(pendulum)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(
    ground, 
    pendulum, 
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))  
)
sys.AddLink(revolute_joint)


app = chronoirr.ChIrrApp(sys, 'Pendulum Simulation', chronoirr.dimension2du(1024, 768))
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(0, 0, 3), chronoirr.vector3df(0, -0.5, 0))  
app.AssetBindAll()
app.AssetUpdateAll()


log_interval = 0.1
next_log_time = 0.0


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    sys.DoStepDynamics(1e-3)  

    
    current_time = sys.GetChTime()
    if current_time >= next_log_time:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {current_time:.3f}, Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}), "
              f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
        next_log_time += log_interval