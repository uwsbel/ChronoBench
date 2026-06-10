import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math






sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


contact_material = chrono.ChContactMaterialNSC()







mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000, True, False)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)

mfloor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(mfloor)



mcrank = chrono.ChBodyEasyBox(1.5, 0.5, 0.5, 1000, True, False)
mcrank.SetPos(chrono.ChVector3d(1, 0, 0))
mcrank.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.2, 0.2))
sys.Add(mcrank)



mrod = chrono.ChBodyEasyBox(4, 0.3, 0.3, 1000, True, False)
mrod.SetPos(chrono.ChVector3d(4, 0, 0))
mrod.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.6, 0.2))
sys.Add(mrod)



mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.4, 0.5, 1000, True, False)
mpiston.SetPos(chrono.ChVector3d(6, 0, 0))
mpiston.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))  
mpiston.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.6))
sys.Add(mpiston)







my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(
    mcrank,
    mfloor,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
)

const_speed = chrono.ChFunctionConst(chrono.CH_PI)
my_motor.SetSpeedFunction(const_speed)
sys.Add(my_motor)



joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(
    mcrank,
    mrod,
    chrono.ChFramed(chrono.ChVector3d(2, 0, 0), chrono.QUNIT)
)
sys.Add(joint_crank_rod)


joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(
    mpiston,
    mrod,
    chrono.ChFramed(chrono.ChVector3d(6, 0, 0), chrono.QUNIT)
)
sys.Add(joint_rod_piston)



joint_piston_truss = chrono.ChLinkLockPrismatic()

joint_piston_truss.Initialize(
    mpiston,
    mfloor,
    chrono.ChFramed(chrono.ChVector3d(6, 0, 0),
                    chrono.QuatFromAngleY(chrono.CH_PI_2))
)
sys.Add(joint_piston_truss)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism Demo')
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


vis.AddSkyBox()


vis.AddCamera(chrono.ChVector3d(3, 4, -6), chrono.ChVector3d(3, 0, 0))


vis.AddTypicalLights()
vis.AddLight(chrono.ChVector3d(5, 8, -5), 12,
             chrono.ChColor(0.8, 0.8, 0.9))





time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    chronoirr.drawGrid(vis, 0.5, 0.5, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0.01, 0),
                                          chrono.QuatFromAngleX(chrono.CH_PI_2)),
                       chrono.ChColor(0.4, 0.4, 0.4), True)

    vis.EndScene()

    
    sys.DoStepDynamics(time_step)