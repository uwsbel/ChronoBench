import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as vehicle
import pychrono.terrain as terrain


sim = chrono.ChSimulation()
sim.SetTimestepper(chrono.ChTimestepper.Type_BDF, 1e-5)
sim.SetMaxPenetrationDepth(1e-3)


app = irr.ChIrrApp(sim, 'Gator Vehicle Simulation', irr.vector2i(800, 600))
app.SetTimestep(0.01)  


terrain_size = 100
terrain = terrain.ChTerrain(terrain_size, terrain_size, 1)
terrain.SetTexture(chrono.GetChronoDataFile('textures/grass.jpg'))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
sim.Add(terrain)


gator = vehicle.ChGator()
gator.SetChassisLocation(chrono.ChVectorD(0, 0.5, 0))
gator.SetChassisOrientation(chrono.Q_from_AngX(chrono.CH_C_PI / 2))
gator.SetContactMethod(vehicle.ChGator.ContactMethod_BULLET)
gator.SetTireModel(vehicle.ChGator.TireModel_TMEASY)
gator.SetChassisBodyVisualizationType(vehicle.ChGator.ChassisBodyVisualizationType_MESH)
gator.SetWheelVisualizationType(vehicle.ChGator.WheelVisualizationType_MESH)
sim.Add(gator)


driver = vehicle.ChGatorDriver(gator)
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBrake(0)


while app.GetDevice().run():
    
    sim.DoStepDynamics(0.01)
    sim.DoStepGraphics(0.01)

    
    if app.GetDevice().isKeyPressed(irr.irrKI_KEY_W):
        driver.SetThrottle(1)
    elif app.GetDevice().isKeyPressed(irr.irrKI_KEY_S):
        driver.SetBrake(1)
    else:
        driver.SetThrottle(0)
        driver.SetBrake(0)

    if app.GetDevice().isKeyPressed(irr.irrKI_KEY_A):
        driver.SetSteering(-1)
    elif app.GetDevice().isKeyPressed(irr.irrKI_KEY_D):
        driver.SetSteering(1)
    else:
        driver.SetSteering(0)

    
    app.BeginScene(True, True, chrono.ChColor(0.3, 0.3, 0.3))
    app.DrawAll()
    app.EndScene()


app.GetDevice().drop()