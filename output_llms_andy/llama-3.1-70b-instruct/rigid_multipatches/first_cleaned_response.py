import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.engine as eng


system = chrono.ChSystemNSC()
contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.9)
contact_material.SetDampingF(0.00001)
contact_material.SetCompliance(0.00001)
contact_material.SetComplianceT(0.00001)
system.SetContactMaterial(contact_material)


veh_hmmwv = veh.HMMWV()
veh_hmmwv.SetChassisVisualization(chrono.ChVisualizationType.MESH)
veh_hmmwv.SetSuspensionVisualization(chrono.ChVisualizationType.MESH)
veh_hmmwv.SetWheelVisualization(chrono.ChVisualizationType.MESH)
veh_hmmwv.SetChassisCollisionShape(chrono.ChCollisionShapeType.BOX)
veh_hmmwv.SetEngineType(eng.ChEngineType.INTERNAL_COMBUSTION)
veh_hmmwv.SetDrivetrainType(eng.ChDrivetrainType.TWO_WHEEL_DRIVE)
veh_hmmwv.SetPosition(chrono.ChVectorD(0, 0.5, 0))
veh_hmmwv.SetEngineMaxTorque(1000)
veh_hmmwv.SetEngineMaxPower(50000)
veh_hmmwv.SetBrakeTorque(1000)
veh_hmmwv.SetSteeringMaxTorque(1000)
system.Add(veh_hmmwv.GetChassisBody())


terrain = chrono.ChBodyEasyBox(system, 100, 10, 100, 1000, True, contact_material)
terrain.SetPosition(chrono.ChVectorD(0, -10, 0))
system.Add(terrain)


patch1 = chrono.ChBodyEasyBox(system, 20, 1, 20, 1000, True, contact_material)
patch1.SetPosition(chrono.ChVectorD(-20, -9, 0))
patch1.SetTextureFile("texture1.jpg")
system.Add(patch1)


patch2 = chrono.ChBodyEasyBox(system, 20, 1, 20, 1000, True, contact_material)
patch2.SetPosition(chrono.ChVectorD(20, -9, 0))
patch2.SetTextureFile("texture2.jpg")
system.Add(patch2)


patch3 = chrono.ChBodyEasyMesh(system, "bump.obj", 1000, True, contact_material)
patch3.SetPosition(chrono.ChVectorD(0, -8, 0))
system.Add(patch3)


patch4 = chrono.ChBodyEasyHeightfield(system, "heightmap.jpg", 100, 100, 1000, True, contact_material)
patch4.SetPosition(chrono.ChVectorD(0, -7, 0))
system.Add(patch4)


driver = veh.ChIrrlichtDriver(veh_hmmwv)
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)


myapplication = chronoirr.ChVisualSystemIrrlicht()
myapplication.SetWindowSize(1024, 768)
myapplication.SetWindowTitle("HMMWV Simulation")
myapplication.SetTypicalLenght(1)
myapplication.SetCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 1, 0))
myapplication.SetCameraUp(chrono.ChVectorD(0, 1, 0))
myapplication.AddTypicalCamera(chrono.ChVectorD(0, 1, -3))
myapplication.AddLightWithShadow(chrono.ChVectorD(0, 10, -10), chrono.ChColor(1, 1, 1), 24, 48, 16, 60)
myapplication.AddSkyBox()
myapplication.AddLogo()
myapplication.AddCameraVerticalAll()
myapplication.AddTypicalLights()
myapplication.AddLight(chrono.ChVectorD(0, 10, -10), chrono.ChColor(1, 1, 1), True)
myapplication.AddFloor(-10)


while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    for substep in range(4):
        system.DoStepDynamics(0.01 / 4)
        driver.Synchronize(0.01 / 4)
    myapplication.EndScene()