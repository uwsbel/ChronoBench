import pychrono as chrono
import pychrono.irrlicht as irr
import math

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# 1. Create the simulation system and add items
m1 = chrono.ChBodyEasyBox(0.5, 0.4, 1.0, 1000, True, True)
m1.SetPosChWorld(chrono.ChVector3d(0, 0, 2))
m1.SetAngleXY(0.2)
m1.SetRotAxis(chrono.ChVector3d(0, 0, 1))
m1.SetName('floor')
m1.GetVisualShape(0).SetMaterial(0.0, 0.0, 0.1)
#m1.EnableCollision(0)
sys = chrono.ChSystemNSC()
sys.Add(m1)

# Create a revolute joint
m2 = chrono.ChBody()
m2.SetMass(10)
m2.SetInertiaXXNNN(1,1,1)
m2.SetPos(chrono.ChVector3d(0,-0.5,0.5))
m2.AddCollisionShape(chrono.ChCollisionShapeBox(m2.GetHalfExtension(), m2.GetHalfExtension(), m2.GetHalfExtension()))
sys.Add(m2)

# Create a revolute joint
m3 = chrono.ChBody()
m3.SetName('pendulum')
m3.SetPos(chrono.ChVector3d(0,0,1))
m3.AddRevoluteConstraint(m2, chrono.ChFramed(chrono.ChVector3d(0,-0.5,0.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1,0,0))))
sys.Add(m3)

# Create a revolute motor
motor = chrono.ChLinkLockRevolute()
motor.SetSpindleSpeed(1)
motor.Initialize(m2, m3)
sys.Add(motor)

# Create a collision shape for the pendulum
shape = chrono.ChCollisionShapeBox(m3.GetHalfExtension(), m3.GetHalfExtension(), m3.GetHalfExtension())
shape.SetNegationMethod(chrono.ChCollisionBoxNegationMethod_BULLET)
m3.AddCollisionShape(shape)

# Create a ground body attached to the chro Chrono body
#ground = chrono.ChBodyEasyBox(100,100,1)
#ground.SetPos(chrono.ChVector3d(0,0,-0.5))
#ground.SetFixed(True)
#ground.GetVisualShape(0).SetMaterial(0.0, 0.0, 0.1)
#sys.Add(ground)

# 2. Create an Irrlicht application to visualize the system
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Pendulum')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.2,0.2,-0.2))
vis.AddTypicalLights()

# Create and attach an FPS driver
driver = irr.ChDriverSimple(vis)
driver.SetChaseCamera(chrono.ChVector3d(0.2,0.2,-0.2), 6.0, 0.5)

# 3. Run the simulation
while vis.Run() :
    vis.BeginScene()
    vis.GetSceneManager().getActiveCamera().setTarget(0.2,0.2,-0.2)
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)

    # Get the current position and velocity of the pendulum
    q = m3.GetRot()
    x = m3.GetPos()
    v = m3.GetVel()
    print ('time =', sys.GetChTime(), ' x =', x, ' v =', v, ' q =', q)