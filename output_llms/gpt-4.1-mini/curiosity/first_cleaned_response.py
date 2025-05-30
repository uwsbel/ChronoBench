import pychrono as chrono
import pychrono.vehicle
import pychrono.irrlicht as chronoirr
import os




chrono.SetChronoDataPath(chrono.GetChronoDataPath())  

system = chrono.ChSystemNSC()  


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  




ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetIdentifier(-1)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(50, 50, 0.5)  

ground.AddVisualShape(ground_shape)


ground.SetPos(chrono.ChVectorD(0, 0, -0.5))


ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(50, 50, 0.5)
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)


mat_ground = chrono.ChMaterialSurfaceNSC()
mat_ground.SetFriction(0.9)
ground.GetCollisionModel().SetDefaultMaterial(mat_ground)

system.Add(ground)











rover = chrono.ChBody()
rover.SetBodyFixed(False)
rover.SetMass(900)  
rover.SetInertiaXX(chrono.ChVectorD(150, 600, 700))  
rover.SetPos(chrono.ChVectorD(0, 0, 0.75))  
rover.GetCollisionModel().ClearModel()
rover.GetCollisionModel().AddBox(1.0, 0.8, 0.25)  
rover.GetCollisionModel().BuildModel()
rover.SetCollide(True)
mat_rover = chrono.ChMaterialSurfaceNSC()
mat_rover.SetFriction(0.8)

rover.GetCollisionModel().SetDefaultMaterial(mat_rover)


chassis_box = chrono.ChBoxShape()
chassis_box.GetBoxGeometry().Size = chrono.ChVectorD(1.0, 0.8, 0.25)
rover.AddVisualShape(chassis_box)
texture_path = os.path.join(chrono.GetChronoDataPath(), "robotlogo.png")
if os.path.isfile(texture_path):
    rover.GetVisualShape(0).SetTexture(texture_path)  

system.Add(rover)


wheel_radius = 0.3
wheel_width = 0.15


wheel_positions = [
    chrono.ChVectorD(0.8, 0.75, 0.3),   
    chrono.ChVectorD(0.8, -0.75, 0.3),  
    chrono.ChVectorD(0.15, 0.75, 0.3),  
    chrono.ChVectorD(0.15, -0.75, 0.3), 
    chrono.ChVectorD(-0.5, 0.75, 0.3),  
    chrono.ChVectorD(-0.5, -0.75, 0.3), 
]

wheels = []
for i, pos in enumerate(wheel_positions):
    wheel = chrono.ChBody()
    wheel.SetMass(20)
    wheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.2, 0.1))
    wheel.SetPos(rover.GetPos() + pos)
    wheel.GetCollisionModel().ClearModel()
    wheel.GetCollisionModel().AddCylinder(wheel_radius, wheel_radius, wheel_width / 2)
    wheel.GetCollisionModel().BuildModel()
    wheel.SetCollide(True)
    mat_wheel = chrono.ChMaterialSurfaceNSC()
    mat_wheel.SetFriction(0.9)
    wheel.GetCollisionModel().SetDefaultMaterial(mat_wheel)

    
    cyl = chrono.ChCylinderShape()
    cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -wheel_width / 2, 0)
    cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, wheel_width / 2, 0)
    cyl.GetCylinderGeometry().rad = wheel_radius
    wheel.AddVisualShape(cyl)

    system.Add(wheel)
    wheels.append(wheel)




motors = []  

for i, wheel in enumerate(wheels):
    
    
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(wheel, rover, chrono.ChCoordsysD(wheel.GetPos(), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
    system.AddLink(joint)

    
    if i in [0, 1]:
        
        steering_rot = chrono.ChBody()
        steering_rot.SetMass(5)
        steering_rot.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
        steering_rot.SetPos(wheel.GetPos())
        steering_rot.SetCollide(False)
        system.Add(steering_rot)

        
        steering_joint = chrono.ChLinkLockRevolute()
        steering_joint.Initialize(steering_rot, rover, chrono.ChCoordsysD(steering_rot.GetPos(), chrono.QIDENT))
        system.AddLink(steering_joint)

        
        wheel_joint = chrono.ChLinkLockRevolute()
        wheel_joint.Initialize(wheel, steering_rot, chrono.ChCoordsysD(wheel.GetPos(), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
        system.AddLink(wheel_joint)

        
        motor = chrono.ChLinkMotorRotationSpeed()
        motor.Set_sp_function(chrono.ChFunction_Const(0))
        motor.Initialize(steering_rot, rover, chrono.ChFrameD(steering_rot.GetPos(), chrono.QIDENT))
        system.AddLink(motor)
        motors.append(motor)




class SteeringDriver:
    def __init__(self, motors):
        self.motors = motors
        self.time = 0

    def update(self, step):
        self.time += step
        
        steering_angle = 0.4 * chrono.sin(0.5 * self.time)

        
        
        for motor in self.motors:
            
            
            motor.Set_sp(0.4 * chrono.cos(0.5 * self.time))





app = chronoirr.ChIrrApp(system, "Curiosity Rover Navigation on Terrain", chronoirr.dimension2du(1280, 720))


app.AddTypicalCamera(chronoirr.vector3df(3, -6, 2))
app.SetCameraVertical(CameraVertical=True)

app.GetCameraManager().MoveCamera(chronoirr.vector3df(0, 0, 0),  
                                  chronoirr.vector3df(0, 0, 0))   


app.AddTypicalLights()
app.SetWindowSize(1280, 720)
app.SetTimestep(0.01)
app.AssetBindAll()
app.AssetUpdateAll()


app.SetShadowIntensity(0.5)
app.SetLightFollowCamera(True)
app.SetShadowUseIrrealtime(True)
app.SetShadowUseTexture(True)
app.SetShadowUseVSM(False)




driver = SteeringDriver(motors)
time_step = 0.01

while app.GetDevice().run():
    driver.update(time_step)
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()