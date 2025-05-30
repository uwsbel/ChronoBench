import os
import math
import chrono
import chrono.vehicle as veh
import chrono.irrlicht as irr





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))






init_pos = chrono.ChVectorD(0, 0.5, 0)
hmmwv = veh.HMMWV_Full(system)
hmmwv.SetInitPosition(chrono.ChCoordsysD(init_pos, chrono.QUNIT))
hmmwv.SetTireType(veh.RigidTire)  
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)





terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))


terrain.SetSoilParameters(
    2e6,        
    0,          
    1.1,        
    17,         
    30,         
    1e7,        
    2e4,        
    3e4         
)


terrain.Initialize(40, 40, 0.1)  


terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 30000)
terrain.SetVisualization(True)





app = veh.ChVehicleIrrApp(hmmwv.GetVehicle(), "HMMWV SCM Demo")
app.AddTypicalLights()
app.AddSkyBox()
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
app.SetTimestep(0.02)





driver = app.GetDriver()
driver.Initialize()





time_step = 0.02
while app.GetDevice().run():
    time = system.GetChTime()
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    driver_inputs = driver.GetInputs()
    
    
    driver.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    app.Synchronize("", driver_inputs)
    
    
    app.Advance(time_step)
    system.DoStepDynamics(time_step)

    
    if not app.GetPaused():
        app.Update()