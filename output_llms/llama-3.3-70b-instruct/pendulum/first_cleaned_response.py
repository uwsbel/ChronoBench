import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.9)
contact_material.SetDampingF(0.2)
contact_material.SetCompliance(1e-9)
contact_material.SetComplianceT(1e-9)
contact_material.SetCoulombFriction(0.9)
contact_material.SetRestitution(0)


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True, contact_material)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)
sys.Add(ground)


pendulum = chrono.ChBodyEasySphere(sys, 1, 10, contact_material)
pendulum.SetPos(chrono.ChVectorD(0, 5, 0))
pendulum.SetMass(10)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
sys.Add(pendulum)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
sys.Add(revolute_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 10, -20))
vis.AddTypicalGround(chrono.ChVectorD(0, 0, 0))
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Simple Pendulum Simulation")
vis.AddCameraTerrainMesh(ground)


time = 0
dt = 0.01
while time < 10:
    sys.DoStepDynamics(dt)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    time += dt
    
    
    if time % 1 < dt:
        print(f"Time: {time:.2f}, Position: {pendulum.GetPos().x:.2f} {pendulum.GetPos().y:.2f} {pendulum.GetPos().z:.2f}, Velocity: {pendulum.GetPos_dt().x:.2f} {pendulum.GetPos_dt().y:.2f} {pendulum.GetPos_dt().z:.2f}")


vis.Run()