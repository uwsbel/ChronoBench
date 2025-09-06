import math as m  
import pychrono as chrno  
import pychrono.fea as fea  
import pychrono.robot  
import pychrono.vehicle  
import pychrono.irrlicht as chronicls  
import pychrono.physics as fea
import pychrono.vehicle as v
import pychrono.robot as r
import pychrono.vehicle as vr


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)
    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0


out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"


sys = chrono.ChSytemSMC()


L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001)


body_trss = chrono.ChBody()
body_trss.SetFixed(True)
sys.AddBody(body_trss)


boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.15)
body_trss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.AddBody(body_crank)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))
myfun = ChFunctionMyFun()
motor.SetTorqueFunction(myfun)
sys.Add(motor)


arm = r.RoboticArm()
arm.SetPosition(chrono.ChVector3d(0, 0, 0))
arm.SetMass(100)
arm.SetVelocity(chrono.ChVector3d(0, 0, 0))
arm.SetAngle(chrono.ChVector3d(0, 0, 0))


vehicle = v.Vehicle()
vehicle.SetPosition(chrono.ChVector3d(0, 0, 0))
vehicle.SetMass(100)
vehicle.SetVelocity(chrono.ChVector3d(0, 0, 0))
vehicle.SetAngle(chrono.ChVector3d(0, 0, 0))


mesh = fea.ChMesh()


beam_wy = 0.12
beam_wz = 0.15


minertia = fea.ChIneritaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChMassSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)


builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.ChVector3d(0, 0, 0))


node_crnkG = builderA.GetLastBeamNodes()[0]
node_crankB = builderA.GetLastBeamNodes()[-1]


constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.Frame(), node_crnkG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)


sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)


rotor = r.Rotor()
rotor.SetPosition(chrono.ChVector3d(0, 0, 0))
rotor.SetMass(50)
rotor.SetVelocity(chrono.ChVector3d(0, 0, 0))
rotor.SetAngle(chrono.ChVector3d(0, 0, 0))


sys.SetSimulation(rotor)
sys.SetSimulationTime(0)
sys.SetSimulationStep(0.001)
sys.SetSimulationLoop(True)