import pychrono as chrono
import pyirrlicht as irr


chrono.Initialize()


driver = irr.createIrrlichtDevice(irr.EDT_OPENGL, 0, 0, 0, 0)
scene = driver.getSceneManager()
context = scene.getVideoDriver()


env = chrono.ChSystemNSC()


hmmwv = chrono.ChBodyEasyBox(0.1, 0.2, 0.3, 1000, True)
env.Add(hmmwv)


hmmwv.SetPos(chrono.ChVectorD(0, 0, 0.15))
hmmwv.SetRot(chrono.ChQuaternionD(chrono.ChVectorD(0, 0, 1), chrono.CH_C_PI_2))


tire = chrono.ChTire()
tire.SetTireType(chrono.ChTire.TIRE_TYPE_PNEUMATIC)
tire.SetTireRadius(0.1)
tire.SetTireWidth(0.2)
tire.SetTirePressure(200000)
tire.SetTireStiffness(100000)
tire.SetTireFriction(0.8)
hmmwv.AddTire(tire)


irr_mesh = irr.CMesh()
irr_mesh.addTriangle(irr.CVector3f(0, 0, 0), irr.CVector3f(0.1, 0, 0), irr.CVector3f(0, 0.2, 0))
irr_node = scene.addMeshSceneNode(irr_mesh, scene.getRootSceneNode(), -1, irr.CVector3f(0, 0, 0), irr.CVector3f(1, 1, 1))
irr_node.setMaterialFlag(irr.EMF_LIGHTING, False)
irr_node.setMaterialTexture(0, driver.getTexture("hmmwv.png"))
hmmwv.AddAsset(irr_node)


soil = chrono.ChMaterialSurfaceNSC()
soil.SetFriction(0.5)
soil.SetDamping(0.1)
soil.SetYoungModulus(1e7)
soil.SetPoissonRatio(0.3)

terrain = chrono.ChTerrain()
terrain.SetMaterial(soil)
terrain.SetMovingPatch(True)
terrain.SetMovingPatchSize(1)
terrain.SetMovingPatchUpdateRate(50)
terrain.SetMovingPatchFollowBody(hmmwv)
env.Add(terrain)


terrain.SetVisualization(chrono.ChTerrain.VISUALIZATION_SINKAGE)


driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBrake(0)


env.SetSolverType(chrono.ChSolverType_SOR)
env.SetSolverMaxIterations(50)
env.SetSolverTolerance(1e-3)


while driver.run():
    env.DoStepDynamics(0.01)

    
    driver.SetSteering(driver.GetInput().getKey('left') - driver.GetInput().getKey('right'))
    driver.SetThrottle(driver.GetInput().getKey('up'))
    driver.SetBrake(driver.GetInput().getKey('down'))

    scene.drawAll()
    driver.sleep(0.02)

chrono.Terminate()