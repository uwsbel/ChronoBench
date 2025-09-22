import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


planet_center_x = 0.33
planet_center_y = 0.25
planet_center_z = 0.05


truss_center = chrono.ChVector3d(0, 0.5, 0.3)


gear_epicyclic = chrono.ChGearEpicyclic()
gear_epicyclic.SetComputeNfromRatio(True)  
gear_epicyclic.SetRatio(2)  
gear_epicyclic.SetPhase(0)  
gear_epicyclic.SetDiametralPitch(40)  


gear_wheel_pit_size = chrono.ChVector3d(.1, .05, .06)


gear1 = chrono.ChSharedBody()
gear1_physical = chrono.ChBody()
gear_epicyclic.MakeGear1(gear1_physical, truss_center + chrono.ChVector3d(-0.20, 0, 0), chrono.ChVector3d(0, 1, 0), gear_wheel_pit_size)
gear1_physical.SetPos_dt(chrono.ChVector3d(0, 0, 0))  
sys.Add(gear1_physical)  
gear1.SetBody(gear1_physical)  


gear2 = chrono.ChSharedBody()
gear2_physical = chrono.ChBody()
gear_epicyclic.MakeGear2(gear2_physical, truss_center + chrono.ChVector3d(+0.12, 0, 0), chrono.ChVector3d(0, 1, 0), gear_wheel_pit_size)
gear2_physical.SetPos_dt(chrono.ChVector3d(0, 0, 0))  
sys.Add(gear2_physical)  
gear2.SetBody(gear2_physical)  


gearS = chrono.ChSharedBody()
gearS_physical = chrono.ChBody()
gear_epicyclic.MakeGearS(gearS_physical, truss_center + chrono.ChVector3d(planet_center_x, planet_center_y, planet_center_z), chrono.ChVector3d(0, 0, 1))
sys.Add(gearS_physical)  
gearS.SetBody(gearS_physical)  


gearP = chrono.ChSharedBody()
gearP_physical = chrono.ChBody()
gear_epicyclic.MakeGearP(gearP_physical, truss_center + chrono.ChVector3d(planet_center_x, planet_center_y, planet_center_z), chrono.ChVector3d(0, 0, 1), 0.06)
sys.Add(gearP_physical)  
gearP.SetBody(gearP_physical)  


gearR = chrono.ChSharedBody()
gearR_physical = chrono.ChBody()
gear_epicyclic.MakeGearR(gearR_physical, truss_center + chrono.ChVector3d(planet_center_x, planet_center_y, planet_center_z), chrono.ChVector3d(0, 0, 1))
sys.Add(gearR_physical)  
gearR.SetBody(gearR_physical)  


conS = chrono.ChSharedBody()
conSP = chrono.ChSharedBody()
conPR = chrono.ChSharedBody()

conS_physical = chrono.ChBody()
conSP_physical = chrono.ChBody()
conPR_physical = chrono.ChBody()

gear_epicyclic.MakeConnections(conS_physical, conSP_physical, conPR_physical, gearP_physical, chrono.ChVector3d(planet_center_x, planet_center_y, planet_center_z), chrono.ChVector3d(0, 0, 1))

sys.Add(conS_physical)  
sys.Add(conSP_physical)  
sys.Add(conPR_physical)  

conS.SetBody(conS_physical)
conSP.SetBody(conSP_physical)
conPR.SetBody(conPR_physical)


mtruss = chrono.ChBodyEasyBox(0.8, 0.1, 0.05, 1000, True, False)
mtruss.SetPos(truss_center)
mtruss.SetFixed(True)  
sys.Add(mtruss)


bar = chrono.ChBodyEasyBox(0.5, 0.05, 0.05, 1000, True, False)
bar.SetPos(truss_center + chrono.ChVector3d(0.15, -0.20, 0))
sys.Add(bar)  


joint_btr = chrono.ChLinkLockRevolute()
joint_btr.Initialize(bar, mtruss, chrono.ChFramed(truss_center + chrono.ChVector3d(0.15, 0, 0), chrono.QUNIT))
sys.Add(joint_btr)


shaft = chrono.ChBodyEasyCylinder(0.02, 0.05, 1000, True, False)
shaft.SetPos(truss_center + chrono.ChVector3d(0.15, -0.1, 0))
shaft.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))
sys.Add(shaft)


joint_bsh = chrono.ChLinkLockRevolute()
joint_bsh.Initialize(bar, shaft, chrono.ChFramed(truss_center + chrono.ChVector3d(0.15, -0.1, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))
sys.Add(joint_bsh)


gear2s = chrono.ChLinkLockGear()
gear2s.Initialize(gear1_physical, shaft)
gear2s.SetRevolute2(chrono.ChFramed(chrono.ChVector3d(truss_center.x + 0.055, truss_center.y - 0.1, truss_center.z), chrono.QUNIT))
gear2s.SetPhase(chrono.CH_PI / 2)
gear2s.SetRotAxis1(chrono.ChVector3d(0, 0, 1))
gear2s.SetRotAxis2(chrono.ChVector3d(1, 0, 0))
gear2s.SetFusion(chrono.ChLinkLockGear.FUSION_ATERROR)
sys.Add(gear2s)


gear_mot = chrono.ChLinkMotorRotationAngle()
gear_mot.Initialize(gearS_physical, mtruss, chrono.ChFramed(truss_center, chrono.QUNIT))
gear_mot.SetAngleFunction(chrono.ChFunctionAngleConst(0))  
gear_mot.SetMotorFunction(chrono.ChFunctionAngleConst(chrono.CH_PI))  
sys.Add(gear_mot)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.5, 0.3), chrono.ChVector3d(0.3, 0.1, 0.05))
vis.AddTypicalLights()


sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-4)