import os
import math
import numpy as np
import chrono as chrono
import chrono.robot as robot
import chrono.irrlicht as irrlicht
import chrono.vehicle as vehicle


os.environ['PATH'] += ';' + os.getcwd()
chrono.SetChronoDataPath('./data')


my_system = chrono.ChSystemNSC()


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


body = chrono.ChBody()
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetMass(100)
body.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
body.EnableCollision(True)


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(body, body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))


t = 0
dt = 0.01
while t < 10:
    my_system.DoStepDynamics(dt)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    t += dt


def report_contact(pA, pB, plane_coord, distance, eff_radius, cforce, ctorque, modA, modB):
    print(f"Contact detected between {pA} and {pB} at {distance} with force {cforce} and torque {ctorque}")


def generate_simulation():
    
    

    
    

    
    

    
    

    
    

    
    


def fea_example():
    
    sys = chrono.ChSystemSMC()
    mesh = chrono.ChMesh()
    msection = chrono.ChBeamSectionEulerAdvanced()
    msection.SetAsRectangularSection(beam_wy, beam_wz)
    msection.SetYoungModulus(0.01e9)
    msection.SetShearModulus(0.01e9 * 0.3)
    msection.SetRayleighDamping(0.000)
    hnode1 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    hnode2 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
    mesh.AddNode(hnode1)
    mesh.AddNode(hnode2)
    belement1 = chrono.ChElementBeamEuler()
    belement1.SetNodes(hnode1, hnode2)
    belement1.SetSection(msection)
    mesh.AddElement(belement1)

def mbs_example():
    
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

def robotics_example():
    
    sys = chrono.ChSystemSMC()
    robot = robot.RoboSimian(sys, True, True)
    robot.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
    driver = robot.RS_Driver(..., ..., ..., True)
    robot.SetDriver(driver)

def vehicle_example():
    
    car = vehicle.ARTcar()
    car.SetContactMethod(contact_method)
    car.SetChassisCollisionType(chassis_collision_type)
    car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    car.Initialize()
    terrain = vehicle.RigidTerrain(car.GetSystem())
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
    terrain.Initialize()


if __name__ == "__main__":
    generate_simulation()