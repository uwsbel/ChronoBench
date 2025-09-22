import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())




system = chrono.ChSystemSMC()


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.GetChronoDataFile("vehicle/terrain/textures/grass.jpg"),
    200, 200, 1.0  
)
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)
patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetPosition(chrono.ChVectorD(0, 0, 0))
patch.SetCollisionEnabled(True)
terrain.Initialize()




initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.Q_from_AngZ(chrono.CH_C_DEG_TO_RAD * 0)
curiosity = veh.Curiosity(system)
curiosity.Initialize(chrono.ChCoordsysD(initLoc, initRot))





driver = veh.ChIrrGuiDriver(
    veh.GetDataPath() + "UI/curiosity_ui.xml"
)
driver.Initialize()


driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)




app = veh.ChWheeledVehicleIrrApp(curiosity.GetVehicle(), 'Curiosity Rover Simulation', irrlicht_device=chronoirr.EIDT_OPENGL)
app.SetSkyBox()
app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
app.AddTypicalLights(chrono.ChVectorD(30, 30, 100), chrono.ChVectorD(-30, -30, 100), 250, 130)
app.AddTypicalCamera(chrono.ChVectorD(1.5, 1.5, 1.2), chrono.ChVectorD(0, 0, 0))
app.SetChaseCamera(curiosity.GetVehicle(), 6.0, 2.0)
app.SetTimestep(0.01)
app.SetShadows(True)


body = curiosity.GetVehicle().GetChassisBody()
body.AddAsset(chrono.ChTexture(chrono.GetChronoDataFile("vehicle/curiosity/textures/curiosity_chassis.jpg")))




app.AssetBindAll()
app.AssetUpdateAll()

while app.GetDevice().run():
    time = system.GetChTime()
    driver.Synchronize(time)
    curiosity.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    system.DoStepDynamics(0.01)