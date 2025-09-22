import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math
import os







change_collision_system = False



def main() : 
    

    
    
    

    
    ground = chrono.ChBodyEasyBox(10,10,1,1000,True,True)
    ground.SetPos(chrono.ChVector3d(0,0,-1))
    ground.SetFixed(True)
    phys_system.Add(ground)

    
    p = 2
    pendulum = chrono.ChBodyEasyTube(0.1, p, 1000, True, True)
    pendulum.SetPos(chrono.ChVector3d(0,0,5))
    pendulum.SetRot(chrono.ChQuaterniond(1,0,0,0))
    pendulum.SetFixed(False)
    phys_system.Add(pendulum)

    bob = chrono.ChBodyEasySphere(0.2, 1000, True, True)
    bob.SetPos(chrono.ChVector3d(0,-p,0))
    bob.SetRot(chrono.ChQuaterniond(1,0,0,0))
    bob.SetMass(30)
    bob.SetInertiaXX(chrono.ChVector3d(10,10,10))
    phys_system.Add(bob)

    
    
    axis = chrono.ChLine3d(chrono.ChVector3d(0,0,0),chrono.ChVector3d(0,0,1))
    joint = chrono.ChLinkLockRevolute()
    joint.SetName('pendulum_joint')
    joint.Initialize(ground, pendulum, axis, chrono.ChFramed(chrono.ChVector3d(0,0,1)))
    phys_system.Add(joint)

    
    limit = chrono.ChLinkLockRevolute.Limit()
    limit.SetAllowCollision(True)
    limit.SetLimits(-math.pi/3, math.pi/3)
    joint.SetLimit(limit)

    
    
    

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(phys_system)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('Pendulum')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2,2,2),chrono.ChVector3d(0,0.5,0))
    vis.AddTypicalLights()

    
    
    

    phys_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    phys_system.SetSolverMaxIterations(150)

    
    
    

    
    print("pendulum length = " + str(p) + "\n")

    
    render_steps = 20

    
    step_number = 0

    
    while vis.Run() :
        time = phys_system.GetChTime()

        
        if (step_number % render_steps == 0) :
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        dist = math.sqrt(bob.GetPos().y+0.5*p)
        angle1 = math.acos((dist/p)+0.5)
        angle2 = math.atan2(bob.GetPos().x, bob.GetPos().z)

        
        log_file = open("angles.txt", "a")
        log_file.write("%f %f %f\n" % (time, angle1, angle2))
        log_file.close()

        
        step_number += 1

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        vis.Advance(step_size)
        phys_system.DoStepDynamics(step_size)









step_size = 1e-3




collision_system = chrono.ChCollisionSystem.Type_BULLET


render_math = True






main()