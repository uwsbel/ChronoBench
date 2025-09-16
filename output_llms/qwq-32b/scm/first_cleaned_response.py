import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import sys


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


app = irr.ChIrrApp(my_system, 'HMMWV on SCM Terrain', irr.dimension2du(1024, 768))
app.SetSymbolscale(0.01)
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0))


vehicle = veh.HMMWV()
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireType(veh.TireModelType.RIGID)
vehicle.SetTireVisualType(veh.VisualizationType_MESH)


init_position = chrono.ChVectorD(0, 0, 1)
init_rotation = chrono.QUNIT
vehicle.Initialize(chrono.ChCoordsysD(init_position, init_rotation))


soil = veh.SoilModelSCM()
soil.SetCohesion(1000)          
soil.SetFrictionAngle(30 * chrono.CH_C_DEG_TO_RAD)
soil.SetRelativeDensity(0.6)
soil.SetThickness(0.5)          

terrain = veh.DeformableTerrain(my_system, chrono.ChCoordsysD(), soil)
terrain.SetSize(100, 100)       
terrain.SetVisualizationType(veh.VisualizationType_MESH)
terrain.SetVisualizationSinkage(True)


initial_patch_pos = vehicle.GetChassis().GetPos()
moving_patch = terrain.AddPatch(initial_patch_pos, 10, 10, 0, 0)
moving_patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))


driver = veh.ChIrrGuiDriver(app)
vehicle.SetDriver(driver)


my_system.SetTimestepperType(chrono.ChTimestepper.Type_EXPLICIT)
my_system.SetSolverMaxIterations(10)
my_system.SetMaxPenetrationRecoverySpeed(5e3)
my_system.SetTimeStep(0.02)  


while app.GetDevice().run():
    time = my_system.GetChTime()
    
    
    driver.Synchronize(time)
    
    
    vehicle.Synchronize(time, driver)
    
    
    chassis_pos = vehicle.GetChassis().GetPos()
    moving_patch.SetPos(chassis_pos)  
    
    
    my_system.DoStepDynamics()
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    app.DoStep()


app.GetDevice().closeDevice()

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import sys


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


app = irr.ChIrrApp(my_system, 'HMMWV on SCM Terrain', irr.dimension2du(1024, 768))
app.SetSymbolscale(0.01)
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0))


vehicle = veh.HMMWV()
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireType(veh.TireModelType.RIGID)
vehicle.SetTireVisualType(veh.VisualizationType_MESH)


init_position = chrono.ChVectorD(0, 0, 1)
init_rotation = chrono.QUNIT
vehicle.Initialize(chrono.ChCoordsysD(init_position, init_rotation))


soil = veh.SoilModelSCM()
soil.SetCohesion(1000)          
soil.SetFrictionAngle(30 * chrono.CH_C_DEG_TO_RAD)
soil.SetRelativeDensity(0.6)
soil.SetThickness(0.5)          

terrain = veh.DeformableTerrain(my_system, chrono.ChCoordsysD(), soil)
terrain.SetSize(100, 100)       
terrain.SetVisualizationType(veh.VisualizationType_MESH)
terrain.SetVisualizationSinkage(True)


initial_patch_pos = vehicle.GetChassis().GetPos()
moving_patch = terrain.AddPatch(initial_patch_pos, 10, 10, 0, 0)
moving_patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))


driver = veh.ChIrrGuiDriver(app)
vehicle.SetDriver(driver)


my_system.SetTimestepperType(chrono.ChTimestepper.Type_EXPLICIT)
my_system.SetSolverMaxIterations(10)
my_system.SetMaxPenetrationRecoverySpeed(5e3)
my_system.SetTimeStep(0.02)  


while app.GetDevice().run():
    time = my_system.GetChTime()
    
    
    driver.Synchronize(time)
    
    
    vehicle.Synchronize(time, driver)
    
    
    chassis_pos = vehicle.GetChassis().GetPos()
    
    moving_patch.SetPos(chassis_pos)
    
    
    my_system.DoStepDynamics()
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    app.DoStep()


app.GetDevice().closeDevice()