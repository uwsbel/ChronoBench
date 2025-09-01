import os
import math
import numpy as np
import chrono as chrono
from chrono import robot, irrlicht, vehicle


os.system('cls' if os.name == 'nt' else 'clear')
math.pi = math.pi
np = np
chrono = chrono
robot = robot
irrlicht = irrlicht
vehicle = vehicle


my_system = chrono.ChSystemNSC()
chrono.SetChronoDataPath('/path/to/data')
material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simulation Window')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


body = chrono.ChBody()
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetMass(100)
body.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
body.SetFixed(True)
body.EnableCollision(True)


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(body, body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))


while vis.Run():
    my_system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


sys.GetContactContainer().ReportAllContacts(my_rep)
sys.GetContactContainer().RegisterAddContactCallback(my_callback)


def generate_simulation():
    
    

    
    

    
    

    
    

    
    

    
    


def beam_simulation():
    sys = chrono.ChSystemSMC()
    mesh = fea.ChMesh()
    msection = fea.ChBeamSectionEulerAdvanced()
    msection.SetAsRectangularSection(beam_wy, beam_wz)
    msection.SetYoungModulus(0.01e9)
    msection.SetShearModulus(0.01e9 * 0.3)
    msection.SetRayleighDamping(0.000)
    hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
    mesh.AddNode(hnode1)
    mesh.AddNode(hnode2)
    belement1 = fea.ChElementBeamEuler()
    belement1.SetNodes(hnode1, hnode2)
    belement1.SetSection(msection)
    mesh.AddElement(belement1)

def shaking_table_simulation():
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    body_table = chrono.ChBody()
    body_table.SetPos(chrono.ChVector3d(0, -size_table_y / 2, 0))
    body_table_shape = chrono.ChVisualShapeBox(size_table_x, size_table_y, size_table_z)
    body_table.AddVisualShape(body_table_shape)
    sys.Add(body_table)
    link_shaker = chrono.ChLinkLockLock()
    link_shaker.Initialize(body_table, body_floor, chrono.ChFramed())
    sys.Add(link_shaker)
    mfunY = chrono.ChFunctionSine(0.001, 1.5)
    link_shaker.SetMotionY(mfunY)

def robosimian_simulation():
    sys = chrono.ChSystemSMC()
    robot = robosimian.RoboSimian(sys, True, True)
    robot.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
    driver = robosimian.RS_Driver(..., ..., ..., True)
    robot.SetDriver(driver)

def artcar_simulation():
    car = veh.ARTcar()
    car.SetContactMethod(contact_method)
    car.SetChassisCollisionType(chassis_collision_type)
    car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    car.Initialize()
    terrain = veh.RigidTerrain(car.GetSystem())
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
    terrain.Initialize()


beam_simulation()
shaking_table_simulation()
robosimian_simulation()
artcar_simulation()


while vis.Run():
    my_system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()