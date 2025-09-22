import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh


chrono.ChEngine.Initialize()


app = irr.IrrlichtApplication()


citybus = veh.CityBus()


citybus.SetChassisBodyPos(chrono.ChVector3d(0, 0, 1))
citybus.SetChassisBodyRot(chrono.ChQuaterniond(1, 0, 0, 0))


terrain = veh.RigidTerrain(citybus.GetSystem())
terrain.SetTerrainMass(1e10)
terrain.SetCoefficientFriction(0.8)
terrain.SetCoefficientRestitution(0.3)


terrain_mat = chrono.ChMaterialSurfaceNSC()
terrain_mat.SetKstatic(2e5)
terrain_mat.SetKdynamic(2e5)
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.3)
terrain.SetMaterial(terrain_mat)


driver = veh.Driver(citybus)


steering = chrono.ChFunction_Constant(0)
throttle = chrono.ChFunction_Constant(0.5)
braking = chrono.ChFunction_Constant(0)


driver.SetSteeringFunction(steering)
driver.SetThrottleFunction(throttle)
driver.SetBrakingFunction(braking)


citybus.GetChassisBody().AddVisualShape(chrono.ChVisualShape_Box(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(2, 1, 0.5), chrono.ChQuaterniond(1, 0, 0, 0), chrono.ChColor(0.5, 0.5, 0.5)))
citybus.GetSystem().GetVisualSystem().Update()


camera = app.GetCamera()
camera.SetPosition(chrono.ChVector3d(5, -10, 5))
camera.SetTarget(chrono.ChVector3d(0, 0, 1))


while app.Run():
    
    citybus.Update(1 / 50)

    
    app.Render(1 / 50)

    
    pos = citybus.GetChassisBody().GetPos()

    
    camera.SetPosition(chrono.ChVector3d(pos.x() + 5, pos.y() - 10, pos.z() + 5))
    camera.SetTarget(pos)

app.Exit()