import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import math as m







var_materials : bool = False


def my_material_callback(material, loc) :
    if loc.y > 0 :
        material.SetColor(irr.SColor(0xff, 0xff, 0xff, 0x80))
    else :
        material.SetColor(irr.SColor(0xff, 0x80, 0x80, 0x80))





step_size = 1e-3
tire_step_size = step_size


tend = 1000


render_step_size = 1.0 / 50  


out_dir = "./CASCADE"










sys = cascade.ChCascadeSystemSMC()


sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sys.SetCollisionMargin(0.001)


ground_mat = chrono.ChContactMaterialSMC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = sys.AddBody(chrono.ChBodyEasyBox(1000, 1000, 1, 1000, True, True, ground_mat))
ground.SetPos(chrono.ChVector3d(0,0,-1))
ground.SetFixed(True)
sys.GetCollisionSystem().AddBox(ground, 1000, 1000, 1)


wall_mat = chrono.ChContactMaterialSMC()
wall_mat.SetFriction(0.8)
wall_mat.SetRestitution(0.0)
wall = sys.AddBody(chrono.ChBodyEasyBox(1, 1000, 50, 1000, True, True, wall_mat))
wall.SetPos(chrono.ChVector3d(-10,0,25))
wall.SetRot(chrono.ChQuaterniond(1,0,0,0))
sys.GetCollisionSystem().AddBox(wall, 1, 1000, 50)


wall2 = sys.AddBody(chrono.ChBodyEasyBox(1, 1000, 50, 1000, True, True, wall_mat))
wall2.SetPos(chrono.ChVector3d(10,0,25))
wall2.SetRot(chrono.ChQuaterniond(1,0,0,0))
sys.GetCollisionSystem().AddBox(wall2, 1, 1000, 50)


vis = cascade.ChCascadeVisualSystemIrrlicht()
vis.SetWindowTitle('SMC')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachSystem(sys)


driver = cascade.ChCascadeDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)


driver.Initialize()






render_steps = m.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

sys.GetSolver().AsIterative().SetMaxIterations(150)

sys.Initialize()


if (out_dir != "./CASCADE"):
    vis.SetOutputPath(out_dir)
    driver.SetOutputPath(out_dir)
    sys.SetOutputPath(out_dir)
    print("Output path: ", out_dir , "\n")

vis.BeginScene()
vis.Render()
vis.EndScene()


if (var_materials):
    print("Materials will change based on location\n")






timer = chrono.ChRealtimeStepTimer()
while vis.Run() :
    time = sys.GetChTime()

    
    if (time >= tend):
        break

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    vis.Synchronize(time, driver_inputs)
    sys.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    vis.Advance(step_size)
    sys.Advance(step_size)

    
    step_number += 1

    
    timer.Spin(step_size)