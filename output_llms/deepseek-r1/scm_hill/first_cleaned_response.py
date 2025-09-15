import os
import math as m
import chrono
from chrono import ChCoordsysD, ChVectorD, QUNIT
import chrono.vehicle as veh
import chrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(ChVectorD(0, 0, -9.81))


hmmwv = veh.HMMWV_Full()
hmmwv.SetInitPosition(ChCoordsysD(ChVectorD(0, 0, 0.5), QUNIT))
hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)


terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(ChVectorD(0, 0, 0), QUNIT))


soil_params = veh.SoilParametersCallback()
soil_params.Kphi = 5301e3
soil_params.Kc = 102e3
soil_params.n = 0.7
soil_params.cohesion = 1.38e3
soil_params.shear_angle = 31.1 * m.pi / 180
soil_params.mu = 0.8
soil_params.Ce = 1.5e-3
terrain.SetSoilParameters(soil_params)


terrain.InitializeFromHeightMap("heightmap.png",  
                                20.0, 10.0,       
                                0.0, 0.5,         
                                0.05)             

terrain.SetPlotType(veh.SCMDeformableTerrain.PlotType_PRESSURE_YIELD, 0, 30000)


app = irr.ChIrrApp(system, "HMMWV on Deformable Terrain", irr.dimension2du(1280, 720))
app.AddTypicalLights()
app.AddCamera(ChVectorD(3, 3, 1.5), ChVectorD(0, 0, 0))
app.AssetBindAll()
app.AssetUpdateAll()


driver = veh.ChIrrGuiDriver(app)
driver.Initialize()


time_step = 0.001
app.SetTimestep(time_step)


while app.GetDevice().run():
    time = system.GetChTime()
    
    
    app.BeginScene()
    app.DrawAll()
    
    
    driver_inputs = driver.GetInputs()
    hmmwv.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    app.Synchronize("", driver_inputs)
    
    
    app.Advance(time_step)
    system.DoStepDynamics(time_step)
    
    app.EndScene()

    
    if real_time_step:
        chrono.ChRealtimeStepTimer.Spin(time_step)