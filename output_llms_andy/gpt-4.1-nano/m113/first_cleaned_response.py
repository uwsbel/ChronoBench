import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('./data/')  
system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(50, 1, 50, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.8)
material.SetRestitution(0.1)
ground.GetMaterialSurface().SetFriction(material.GetFriction())
ground.GetMaterialSurface().SetRestitution(material.GetRestitution())



m113 = veh.M113_Vehicle(system)
m113.SetTireType(veh.E TireType_Simple)
m113.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
m113.SetChassisVisualizationType(veh.ChassisVisualizationType_PRIMITIVES)
m113.SetTireVisualizationType(veh.TireVisualizationType_PRIMITIVES)


initial_pos = chrono.ChVectorD(0, 0.5, 0)
initial_rot = chrono.Q_from_AngX(0)
m113.GetChassisBody().SetPos(initial_pos)
m113.GetChassisBody().SetRot(initial_rot)


driver = veh.ChIrrGuiDriver()
driver.Initialize(system)
driver.SetInputMode(veh.ChIrrGuiDriver.XYZ)
driver.SetThrottleDelta(0.1)
driver.SetSteeringDelta(0.02)


application = irr.ChIrrApp(system, 'PyChrono M113 Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(0, 3, -10))
application.SetContactsDrawMode(irr.ChIrrTools contactsDrawMode.contactsDrawAll)
application.AssetBindAll()
application.AssetUpdateAll()


time_step = 1e-3
while application.GetDevice().run():
    
    application.BeginScene()
    application.DrawAll()

    
    driver.GetControlInputs()

    
    driver.Synchronize(time_step)
    
    m113.Synchronize(time_step, chrono.VNULL, driver.GetSteering(), driver.GetThrottle(), 0)
    
    m113.Advance(time_step)

    
    application.DoStep();

    
    application.EndScene()