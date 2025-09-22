import os
import math
import numpy as np
import chrono as chrono
import chrono.robot as robot
import chrono.irrlicht as irrlicht
import chrono.vehicle as vehicle


os.environ['PATH'] += ';' + os.path.dirname(chrono.__file__)
import math
import numpy as np
import chrono as chrono
import chrono.robot as robot
import chrono.irrlicht as irrlicht
import chrono.vehicle as vehicle


sys = chrono.ChSystemSMC()


material = chrono.ChContactMaterialSMC()
material.SetFriction(0.5)
material.SetDampingF(0.1)
material.SetCompliance(0.01)
sys.SetDefaultContactMaterial(material)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simulation Window')
vis.Initialize()


body_beam = chrono.ChBody()
body_beam.SetPos(chrono.ChVector3d(0, 0, 0))
body_beam_shape = chrono.ChVisualShapeBox(1, 1, 10)
body_beam.AddVisualShape(body_beam_shape)
sys.Add(body_beam)


joint_flywheel = chrono.ChLinkLockRevolute()
joint_flywheel.Initialize(body_beam, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 5), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
sys.Add(joint_flywheel)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_beam, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 5), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
motor.SetSpeedFunction(chrono.ChFunctionSine(0.001, 1.5))
sys.Add(motor)


sys.GetContactContainer().ReportAllContacts(chrono.ReportContactCallback())


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


def generate_simulation():
    
    sys = chrono.ChSystemSMC()
    body_beam = chrono.ChBody()
    joint_flywheel = chrono.ChLinkLockRevolute()
    motor = chrono.ChLinkMotorRotationSpeed()
    
    
    sys.Add(body_beam)
    joint_flywheel.Initialize(body_beam, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 5), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
    sys.Add(joint_flywheel)
    motor.Initialize(body_beam, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 5), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
    motor.SetSpeedFunction(chrono.ChFunctionSine(0.001, 1.5))
    sys.Add(motor)
    
    
    sys.Validate()
    
    
    
    while vis.Run():
        sys.DoStepDynamics(0.01)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


def beam_simulation():
    
    sys = chrono.ChSystemSMC()
    mesh = chrono.ChMesh()
    msection = chrono.ChBeamSectionEulerAdvanced()
    msection.SetAsRectangularSection(1, 1)
    msection.SetYoungModulus(0.01e9)
    msection.SetShearModulus(0.01e9 * 0.3)
    msection.SetRayleighDamping(0.000)
    hnode1 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    hnode2 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(10, 0, 0)))
    mesh.AddNode(hnode1)
    mesh.AddNode(hnode2)
    belement1 = chrono.ChElementBeamEuler()
    belement1.SetNodes(hnode1, hnode2)
    belement1.SetSection(msection)
    mesh.AddElement(belement1)
    
    
    sys.Add(mesh)
    
    
    sys.Validate()
    
    
    
    while vis.Run():
        sys.DoStepDynamics(0.01)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

def mbs_simulation():
    
    sys = chrono.ChSystemNSC()
    body_table = chrono.ChBody()
    body_table.SetPos(chrono.ChVector3d(0, -1, 0))
    body_table_shape = chrono.ChVisualShapeBox(1, 1, 1)
    body_table.AddVisualShape(body_table_shape)
    sys.Add(body_table)
    link_shaker = chrono.ChLinkLockLock()
    link_shaker.Initialize(body_table, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
    sys.Add(link_shaker)
    mfunY = chrono.ChFunctionSine(0.001, 1.5)
    link_shaker.SetMotionY(mfunY)
    
    
    sys.Add(link_shaker)
    
    
    sys.Validate()
    
    
    
    while vis.Run():
        sys.DoStepDynamics(0.01)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

def vehicle_simulation():
    
    car = chrono.vehicle.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_SMC)
    car.SetChassisCollisionType(chrono.ChCollisionType_BOX)
    car.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
    car.Initialize()
    terrain = chrono.vehicle.RigidTerrain(car.GetSystem())
    patch = terrain.AddPatch(chrono.ChMaterialSurface(chrono.ChColor(1, 0, 0), 0.5), chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)), 10, 10)
    terrain.Initialize()
    
    
    sys.Add(car)
    sys.Add(terrain)
    
    
    sys.Validate()
    
    
    
    while vis.Run():
        sys.DoStepDynamics(0.01)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


generate_simulation()