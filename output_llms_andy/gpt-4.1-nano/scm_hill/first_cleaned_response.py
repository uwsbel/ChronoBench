import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()


terrain = veh.SCMDeformableTerrain(sys)

terrain.SetSoilParameters(veh.SCMDeformableTerrain.SoilParameters(
    friction_angle=30 * chrono.CH_C_DEG_TO_RAD,
    adhesion=0.1,
    cohesion=100,
    roll_angle=15 * chrono.CH_C_DEG_TO_RAD,
    young_modulus=1e7,
    poisson_ratio=0.3,
    shear_modulus=3.8e6,
    damping=0.0
))

height_map = [0.0 for _ in range(100)]  
terrain.Initialize(height_map, 1.0, 100, 1.0)


hmmwv = veh.HMMWV_IV()
hmmwv.SetChassisCollisionType(veh.CollisionType_PRIM)
hmmwv.SetInitPosition(chrono.ChVectorD(0, 0.5, 0))
hmmwv.SetInitFwdVelocity(20)  
sys.Add(hmmwv)


driver = veh.ChIrrGuiDriver()
driver.Initialize(hmmwv.GetVehicle())
driver.GetInputMode()  





vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowTitle('PyChrono HMMWV on SCM Terrain')
vis.Initialize()

vis.GetActiveCamera().SetPosition(chrono.ChVectorD(-50, 20, 0))
vis.GetActiveCamera().SetTarget(chrono.ChVectorD(0, 0, 0))

vis.AddLightWithShadow(chrono.ChVectorD(0, 100, 0), chrono.ChVectorD(0, -1, 0), 100, 4, 0.1, 10)


time_step = 1e-3  
max_time = 20  
current_time = 0.0


while vis.Run():
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver.Synchronize(current_time)
    
    terrain.Synchronize(current_time)
    
    hmmwv.Synchronize(current_time, chrono.ChVectorD(0, 0, 0), driver.GetInput())
    
    terrain.Advance(time_step)
    
    hmmwv.Advance(time_step)
    
    driver.Advance(time_step)

    
    sys.DoStepDynamics(time_step)

    current_time += time_step
    if current_time >= max_time:
        break


vis.GetDevice().close()