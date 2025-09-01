import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irrlicht
import pychrono.vehicle as vehicle


os.chdir('/path/to/project')
import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irrlicht
import pychrono.vehicle as vehicle


my_system = chrono.ChSystemNSC()
chrono.SetChronoDataPath('/path/to/data')


material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)


vis = chronoirr.ChVisualSystemIrrlicht()
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
rev_joint.Initialize(body1, body2, chrono.ChFramed())


sys.DoStepDynamics(0.01)
vis.Run()
vis.BeginScene()
vis.Render()
vis.EndScene()


contact_callback = chrono.ReportContactCallback()
sys.GetContactContainer().RegisterAddContactCallback(contact_callback)



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


sys = chrono.ChSystemSMC()
robot = robosimian.RoboSimian(sys, True, True)
robot.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
driver = robosimian.RS_Driver(..., ..., ..., True)
robot.SetDriver(driver)


car = veh.ARTcar()
car.SetContactMethod(contact_method)
car.SetChassisCollisionType(chassis_collision_type)
car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
car.Initialize()
terrain = veh.RigidTerrain(car.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
terrain.Initialize()


lidar = chrono.ChSensorLidar()
lidar.SetNoiseFilter(0.1)
lidar.SetVisualization(True)
lidar.SetSaveOptions(True)
lidar.SetPosition(chrono.ChVector3d(0, 0, 0))
lidar.SetRotation(chrono.QUNIT)
sys.Add(lidar)
lidar_data = lidar.GetBuffer()


while vis.Run():
    sys.DoStepDynamics(0.01)
    lidar_data = lidar.GetBuffer()
    print(lidar_data)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()