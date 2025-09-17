import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


body_ground = chrono.ChBodyEasy()
body_ground.SetBodyFixed(True)
body_ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
body_ground.SetShapeBoxes(0.5, 0.1, 1)
body_ground.SetCollide(True)
system.Add(body_ground)




body_rover = chrono.ChBodyEasy()
body_rover.SetPos(chrono.ChVectorD(0, 0.5, 0))
body_rover.SetShapeBoxes(0.3, 0.2, 0.5)
body_rover.SetCollide(True)
body_rover.SetMass(5)  
system.Add(body_rover)




class SteeringController:
    def __init__(self, body, steering_angle_limit=30):
        self.body = body
        self.steering_angle_limit = steering_angle_limit
        self.steering_angle = 0.0

    def SetSteeringAngle(self, angle):
        self.steering_angle = chrono.ChMath.Clamp(angle, -self.steering_angle_limit, self.steering_angle_limit)

    def Update(self, time_step):
        
        torque = chrono.ChVectorD(0, 0, self.steering_angle * 10)  
        self.body.ApplyTorque(torque)

steering_controller = SteeringController(body_rover)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5))
vis.AddTypicalLights()
vis.AddLogo()


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if vis.GetSystem().GetChTime() < 10:  
        if vis.GetSystem().GetChTime() > 5:
            steering_controller.SetSteeringAngle(10)
        else:
            steering_controller.SetSteeringAngle(-10)
    else:
        steering_controller.SetSteeringAngle(0)

    steering_controller.Update(time_step)

    system.DoStepDynamics(time_step)