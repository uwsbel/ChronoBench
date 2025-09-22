import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath("../chrono/data/")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


terrain = chrono.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), 100, 100)
terrain.SetMu(0.5)
terrain.SetCohesion(1000)
terrain.SetRestitution(0.1)
terrain.SetPlasticity(0.0)
terrain.SetDensity(1000)
terrain.SetThickness(0.1)


terrain.SetVisualizationType(chrono.SCMDeformableTerrain.VISUALIZATION_SINKAGE)
terrain.SetSinkageColorMap(chrono.SCMDeformableTerrain.COLORMAP_JET)


hmmwv = veh.HMMWV(system)


initial_position = chrono.ChVectorD(0, 1, 0)
initial_rotation = chrono.ChQuaternionD(1, 0, 0, 0)
hmmwv.SetPos(initial_position)
hmmwv.SetRot(initial_rotation)


for wheel in hmmwv.GetWheels():
    tire = wheel.GetTire()
    tire.SetModel(chrono.vehicle.Tire.Model.RIGID)


for body in hmmwv.GetBodies():
    if body.GetCollisionModel() is not None:
        body.GetCollisionModel().AddVisualization(chrono.ChColor(0.5, 0.5, 0.5))
    


patch = chrono.SCMDeformableTerrain.MovingPatch(terrain)
patch.SetChassis(hmmwv.GetChassis())
patch.SetRadius(1.5)
patch.SetResolution(32)


driver = veh.ChIrrlichtDriver(hmmwv)
driver.SetSteeringDelta(0.01)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddTypicalLights()


time_step = 0.005
vis.SetTimeStep(time_step)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)
    driver.Synchronize(time_step)