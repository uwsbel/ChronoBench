import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.scm as scm


chrono.SetChronoDataPath("path/to/chrono/data/")
myapp = chronoirr.ChIrrApp(__file__, "HMMWV on SCM Deformable Terrain", chronoirr.dimension2du(800, 600))
application = veh.ChPart("military")
myapp.AddTypicalSky()
myapp.AddTypicalLights()
myapp.AddTypicalCamera(chronoirr.vector3df(0, 2, -5))


terrain = scm.ChSCMTerrain()
terrain.SetSoilParameters(scm.ChSCMSoilParameters(scm.ChSCMSoilType.SAND, 0.1, 0.05, 0.01, 0.001, 0.1, 0.1))
terrain.SetMovingPatchEnabled(True)
terrain.SetMovingPatchSize(10, 10)
terrain.SetSinkageVisualization(True)
myapp.Add(terrain)


vehicle = veh.ChPart("HMMWV")
vehicle.SetPosition(chrono.ChVectorD(0, 1, 0))
vehicle.SetOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.SetMass(2500)
vehicle.SetInertia(chrono.ChMatrix33D(1, 0, 0, 0, 1, 0, 0, 0, 1))


tire_model = veh.ChRigidTireModel()
tire_model.SetTireParameters(veh.ChTireParameters(1.2, 0.3, 0.1, 0.05, 0.01))
vehicle.AddTireModel(tire_model)


vehicle.AddVisualization(veh.ChVisualizationType.MESH)


driver = veh.ChIrrNodeDriver()
driver.Initialize(myapp)
driver.SetSteeringIncrement(0.1)
driver.SetThrottleIncrement(0.1)
driver.SetBrakingIncrement(0.1)
vehicle.AddDriver(driver)


myapp.SetTimestep(0.02)
myapp.SetVideoframe(50)
myapp.AssetBindAll()
myapp.AssetUpdateAll()


while myapp.GetDevice().run():
    myapp.BeginScene()
    myapp.DrawAll()
    myapp.DoStep()
    myapp.EndScene()