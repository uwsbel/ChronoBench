import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'data/'))
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


vehicle = veh.MAN_5t_Vehicle()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


tire_pressure = 100000  
for axle in vehicle.GetAxles():
    tire_l = axle.m_wheels[0].GetTire()
    tire_r = axle.m_wheels[1].GetTire()
    tire_l.SetVisualizationType(veh.VisualizationType_MESH)
    tire_r.SetVisualizationType(veh.VisualizationType_MESH)
    tire_l.SetPressure(tire_pressure)
    tire_r.SetPressure(tire_pressure)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                        chrono.ChVectorD(100, 100, 1), 
                        chrono.ChVectorD(0, 0, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-5, 0, 2), chrono.ChVectorD(0, 0, 0.5))
vis.AddLightWithShadow(chrono.ChVectorD(-10, 10, 10), chrono.ChVectorD(0, 0, 0), 50, 10, 50, 8192)


driver = veh.InteractiveDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


time_step = 0.002
real_time_step = 0.02
time = 0.0

while vis.Run():
    time = system.GetChTime()
    
    
    driver.Synchronize(time)
    
    
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    vis.Synchronize(time, driver.GetInputs())
    
    
    system.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    time += real_time_step