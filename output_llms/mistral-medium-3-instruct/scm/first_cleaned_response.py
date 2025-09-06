import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.scm as scm
import numpy as np
import math






step_size = 0.01


t_end = 100


vis_rate = 50


terrain_length = 100  
terrain_width = 50    






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))






hmmwv_params = veh.HMMWV_VehicleParams()
hmmwv_params.chassis.mass = 2700
hmmwv_params.chassis.inertia = chrono.ChVectorD(1000, 1000, 1000)
hmmwv_params.chassis.com = chrono.ChVectorD(0, 0, 0.5)


hmmwv = veh.HMMWV(system)
hmmwv.Initialize(hmmwv_params)


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.8), chrono.QUNIT))
hmmwv.SetInitFwdVel(10)


for i in range(hmmwv.GetNumberAxles()):
    hmmwv.GetAxle(i).m_wheels[0].SetVisualizationType(veh.VisualizationType_MESH)
    hmmwv.GetAxle(i).m_wheels[1].SetVisualizationType(veh.VisualizationType_MESH)






soil_params = scm.ChSoilParameters()
soil_params.elastic_k = 1e7
soil_params.damping_r = 1e4
soil_params.cohesion = 1000
soil_params.friction_angle = 30
soil_params.bev_angle = 45


terrain = scm.ChSCMTerrain(system)
terrain.SetSoilParameters(soil_params)
terrain.SetPlasticityFlag(True)
terrain.SetFixedPatch(True)
terrain.SetPatchSize(10, 10)  
terrain.SetPatchResolution(0.1, 0.1)  


terrain.Initialize(chrono.ChVectorD(terrain_length, terrain_width, 0.5),
                  chrono.ChVectorD(terrain_length/2, terrain_width/2, 0))


terrain.SetPlotSinkage(True)
terrain.SetPlotType(scm.PlotType::PRESSURE)






driver = veh.ChDriverHMMWV(system)
driver.Initialize(hmmwv)


driver.SetSteeringFunction(veh.ChFunction_Const(0))  
driver.SetThrottleFunction(veh.ChFunction_Const(0.2))  
driver.SetBrakingFunction(veh.ChFunction_Const(0))  






vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 2))
vis.AddTypicalLights()
vis.AttachSystem(system)


vis.SetSymbolScale(0.5)
vis.SetShadows(True)






real_time = True
time_step = step_size
sim_time = 0


while vis.Run() and sim_time < t_end:
    
    time = system.GetChTime()

    
    if vis.GetDevice().GetKeyState(chrono.irr::KEY_KEY_W):
        driver.SetThrottle(0.5)
    elif vis.GetDevice().GetKeyState(chrono.irr::KEY_KEY_S):
        driver.SetThrottle(-0.3)  
    else:
        driver.SetThrottle(0)

    if vis.GetDevice().GetKeyState(chrono.irr::KEY_KEY_A):
        driver.SetSteering(-0.3)
    elif vis.GetDevice().GetKeyState(chrono.irr::KEY_KEY_D):
        driver.SetSteering(0.3)
    else:
        driver.SetSteering(0)

    
    chassis_pos = hmmwv.GetChassis().GetPos()
    terrain.SetPatchPosition(chrono.ChVectorD(chassis_pos.x, chassis_pos.y, 0))

    
    system.DoStepDynamics(time_step)

    
    if real_time:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        vis.Advance(time_step)

    
    sim_time += time_step

    
    if math.fmod(time, 1) < time_step:
        print("Time: ", time, "s")
        print("Vehicle position: ", chassis_pos)
        print("Vehicle speed: ", hmmwv.GetChassisBody().GetPos_dt().Length())
        print("----------------------------------")





vis.DetachSystem()
vis.Close()