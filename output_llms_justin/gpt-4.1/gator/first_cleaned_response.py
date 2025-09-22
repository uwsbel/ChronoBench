import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())


step_size = 0.01
render_step_size = 1.0 / 50  


initLoc = chrono.ChVectorD(0, 0, 0.5)
initYaw = chrono.CH_C_PI / 12  
contact_method = chrono.ChContactMethod_NSC




gator = veh.Gator()
gator.SetContactMethod(contact_method)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(initLoc, chrono.Q_from_AngZ(initYaw)))
gator.SetTireType(veh.TireType_TMEASY)
gator.SetTireStepSize(step_size)
gator.SetVisualizationTypeChassis(veh.VisualizationType_MESH)
gator.SetVisualizationTypeSuspension(veh.VisualizationType_MESH)
gator.SetVisualizationTypeSteering(veh.VisualizationType_MESH)
gator.SetVisualizationTypeWheel(veh.VisualizationType_MESH)
gator.SetVisualizationTypeTire(veh.VisualizationType_MESH)
gator.Initialize()




terrain = veh.RigidTerrain(gator.GetSystem())
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    length=60.0, width=30.0
)
patch.SetContactFrictionCoefficient(0.9)
patch.SetRestitutionCoefficient(0.01)
patch.SetMaterialSurface(contact_method)

patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()




driver = veh.ChIrrGuiDriver(gator.GetVehicle())
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()




vis = veh.ChWheeledVehicleIrrApp(gator.GetVehicle(), "PyChrono Gator Demo", irr.dimension2du(1024,768))
vis.SetSkyBox()
vis.AddTypicalLights()
vis.AddTypicalLogo()
vis.AddTypicalCamera(chrono.ChVectorD(0.0, 2.0, 1.75), chrono.ChVectorD(0, 0, 0.5))
vis.AttachVehicle(&gator.GetVehicle())




realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = int(render_step_size / step_size)
step_number = 0

while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(gator.GetSystem().GetChTime())
    terrain.Synchronize(gator.GetSystem().GetChTime())
    gator.Synchronize(gator.GetSystem().GetChTime(), driver_inputs, terrain)
    vis.Synchronize("Gator Demo", driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    step_number += 1