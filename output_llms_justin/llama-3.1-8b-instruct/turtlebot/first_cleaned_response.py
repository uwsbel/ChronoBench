import chrono as c
import irrlicht
import time


c.init_data()
c.init_system()
c.init_irrlicht()


g = c.VECT3(0, 0, -9.81)
sys = c.ChSystem()
sys.Set_G(c.MAT3(c.VECT3(1, 0, 0), c.VECT3(0, 1, 0), c.VECT3(0, 0, 1)))


ground = c.ChBody()
ground.SetBodyType(c.CH_BTS_STATIC)
ground.SetDensity(0)
ground.SetPos(c.VECT3(0, 0, 0))
ground.SetRot(c.ROT3(c.PI/2, 0, 0))
sys.Add(ground)


robot = c.ChBody()
robot.SetBodyType(c.CH_BTS_RIGID)
robot.SetDensity(10)
robot.SetPos(c.VECT3(0, 0, 1))
robot.SetRot(c.ROT3(0, 0, 0))
sys.Add(robot)


motor = c.ChMotor()
motor.SetForce(10)
motor.SetTorque(10)
robot.AddMotor(motor)
joint = c.ChJoint()
joint.SetType(c.CH_JNT_REVOLUTE)
joint.SetAnchor(c.VECT3(0, 0, 0))
joint.SetAxis(c.VECT3(0, 0, 1))
robot.AddJoint(joint)


motor = c.ChMotor()
motor.SetForce(10)
motor.SetTorque(10)
robot.AddMotor(motor)
joint = c.ChJoint()
joint.SetType(c.CH_JNT_REVOLUTE)
joint.SetAnchor(c.VECT3(0, 0, 0))
joint.SetAxis(c.VECT3(0, 0, 1))
robot.AddJoint(joint)


device = c.ChIrrApp(sys, c.dimension2du(800, 600), c.EDS_COLOR_FLAG, c.EDS_BACK_FACE_COLOR_FLAG, True)
device.SetTimestep(1.0/60.0)
device.SetCameraRotation(c.VECT3(0, 0, 0))
device.SetCameraPosition(c.VECT3(0, 0, 10))
device.SetLightAmbient(c.VECT3(0.2, 0.2, 0.2))
device.SetLightDiffuse(c.VECT3(0.8, 0.8, 0.8))
device.SetLightSpecular(c.VECT3(1, 1, 1))


t = 0
while True:
    
    sys.DoStep()
    device.BeginScene()
    device.DrawAll()
    device.EndScene()
    time.sleep(0.01)
    
    if t > 10:
        joint.SetAngle(c.PI/2)
    
    elif t > 20:
        joint.SetAngle(-c.PI/2)
    t += 1