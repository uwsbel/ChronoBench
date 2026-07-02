import pychrono as chrono
from pychrono import vehicle, irrlicht as irr
from pychrono.vehicle import ChVehicle, ChWheeledVehicle, ChTire
import os


sys = vehicle.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


truck = vehicle.MAN_10t(sys)
truck.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
truck.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
truck.SetChassisCollisionType(vehicle.CollisionType_SIMPLE)
truck.Initialize()


terrain = vehicle.RigidTerrain(sys)
terrain.SetLength(200)
terrain.SetWidth(200)
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.AddVisualisationBox()
terrain.AddLogo(chrono.GetChronoDataFile("logo_chrono.png"), chrono.ChVectorD(0, 0, 1))
terrain.SetFriction(0.9)
terrain.SetRestitution(0.01)
terrain.Initialize()


tire_file = chrono.GetChronoDataFile("vehicle/tires/TMeasy.json")
for axle in truck.GetAxles():
    for side in vehicle.LEFT, vehicle.RIGHT:
        wheel = axle.GetSpindle(side)
        tire = vehicle.TMeasyTire(sys)
        tire.Load(tire_file)
        
        if axle.GetIndex() == 0:
            tire.SetDriven(True)
        tire.Initialize(wheel)
        truck.AddTire(tire)


app = irr.ChIrrApp(sys, "MAN 10t Truck Simulation", irr.dimension2du(1280, 720))
app.SetSkyBox()
app.SetCamera(chrono.ChVectorD(0, -10, 2), chrono.ChVectorD(0, 0, 1))
app.AddLightDirectional()
app.SetChaseCamera(chrono.ChVectorD(0, 0, 1), 10, 5)
app.SetShowInfos(True)


driver = vehicle.ChInteractiveDriverIRR(truck, app.GetDevice())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)
driver.SetMaxSteering(0.6)
driver.SetMaxThrottle(1.0)
driver.SetMaxBraking(1.0)


sys.SetTimestep(0.001)
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    driver.Update()
    sys.DoStepDynamics(0.001)
    app.EndScene()