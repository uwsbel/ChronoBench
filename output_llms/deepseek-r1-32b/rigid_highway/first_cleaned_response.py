importchrono
import pychrono as chrono
import pychrono irrlicht as chronovisualization


vis = chronovisualization.ChVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on Custom Terrain')
vis.SetWindowSize(1024, 768)
vis.SetCameraPosition(chrono.ChVectorD(0, 5, 10))
vis.SetCameraRotation(chrono.ChVectorD(0, -45, 0))


hmmwv = chrono.ChHMMWV()
hmmwv.SetPos(chrono.ChVectorD(0, 0, 1))  
hmmwv.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  


hmmwv.GetVehicle().SetContactMethod(chrono.ChContactMethod_NSM)
hmmwv.GetVehicle().SetTireModel(chrono.ChTireModelType_TMEASY)


for component in hmmwv.GetComponents():
    vis.Add(component)


terrain = chrono.ChBody()
terrain.SetBodyFixed(True)
terrain.SetCollide(True)


collision_mesh = chrono.ChTriangleMesh()
collision_mesh.Load('Highway_col.obj')
terrain.GetCollisionModel().AddTriangleMesh(collision_mesh)
terrain.GetCollisionModel().SetMaterial(chrono.ChMaterialSurface())

visual_mesh = chrono.ChTriangleMesh()
visual_mesh.Load('Highway_vis.obj')
vis.Add(chrono.ChVisualShape(visual_mesh))


hmmwv.GetSystem().AddBody(terrain)


driver = chrono.ChIrrGuiDriver()
hmmwv.SetDriver(driver)


driver.SetSteeringCallback(lambda: 0.5 * (chrono.GetInput().GetKeyDown(chrono.KEY_RIGHT) - chrono.GetInput().GetKeyDown(chrono.KEY_LEFT)))
driver.SetThrottleCallback(lambda: chrono.GetInput().GetKeyDown(chrono.KEY_UP))
driver.SetBrakingCallback(lambda: chrono.GetInput().GetKeyDown(chrono.KEY_SPACE))


vis.Begin()
while vis.Run():
    driver.Update()
    hmmwv.GetSystem().DoStepDynamics(1/50)
    vis.Render()
vis.End()