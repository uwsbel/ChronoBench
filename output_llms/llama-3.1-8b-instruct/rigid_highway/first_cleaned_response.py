import chrono as chrono
import irrlicht


chrono.Initialize()
core = chrono.Core()
core.SetTimestep(1/50.0)  


terrain_col_mesh = chrono.CreateCollisionMeshFromOBJ("Highway_col.obj")
terrain_vis_mesh = chrono.CreateVisualMeshFromOBJ("Highway_vis.obj")


terrain = chrono.ChBodyEasyBox(100, 1, 1)
terrain.SetBodyFixed(True)
terrain.SetCollisionMesh(terrain_col_mesh)
terrain.SetVisualMesh(terrain_vis_mesh)
core.Add(terrain)


hmmwv_mass = 5000
hmmwv_length = 5
hmmwv_width = 2
hmmwv_height = 2


chassis = chrono.ChBodyEasyBox(hmmwv_length, hmmwv_width, hmmwv_height)
chassis.SetMass(hmmwv_mass)
chassis.SetPos(chrono.Vect(0, 0, 0))
chassis.SetRot(chrono.QfromEuler(0, 0, 0))
chassis.SetBodyFixed(False)
core.Add(chassis)


tire_radius = 0.5
tire_mass = 100
tire_stiffness = 1000000


tire_front_left = chrono.ChBodyEasyCylinder(tire_radius, tire_mass)
tire_front_left.SetPos(chrono.Vect(-hmmwv_length/2, hmmwv_width/2, 0))
tire_front_left.SetRot(chrono.QfromEuler(0, 0, 0))
tire_front_left.SetBodyFixed(False)
tire_front_left.SetContactMethod(chrono.ContactMethod_2DCOLLISION)
tire_front_left.SetTMEasyTire(tire_radius, tire_stiffness)
core.Add(tire_front_left)

tire_front_right = chrono.ChBodyEasyCylinder(tire_radius, tire_mass)
tire_front_right.SetPos(chrono.Vect(-hmmwv_length/2, -hmmwv_width/2, 0))
tire_front_right.SetRot(chrono.QfromEuler(0, 0, 0))
tire_front_right.SetBodyFixed(False)
tire_front_right.SetContactMethod(chrono.ContactMethod_2DCOLLISION)
tire_front_right.SetTMEasyTire(tire_radius, tire_stiffness)
core.Add(tire_front_right)

tire_rear_left = chrono.ChBodyEasyCylinder(tire_radius, tire_mass)
tire_rear_left.SetPos(chrono.Vect(hmmwv_length/2, hmmwv_width/2, 0))
tire_rear_left.SetRot(chrono.QfromEuler(0, 0, 0))
tire_rear_left.SetBodyFixed(False)
tire_rear_left.SetContactMethod(chrono.ContactMethod_2DCOLLISION)
tire_rear_left.SetTMEasyTire(tire_radius, tire_stiffness)
core.Add(tire_rear_left)

tire_rear_right = chrono.ChBodyEasyCylinder(tire_radius, tire_mass)
tire_rear_right.SetPos(chrono.Vect(hmmwv_length/2, -hmmwv_width/2, 0))
tire_rear_right.SetRot(chrono.QfromEuler(0, 0, 0))
tire_rear_right.SetBodyFixed(False)
tire_rear_right.SetContactMethod(chrono.ContactMethod_2DCOLLISION)
tire_rear_right.SetTMEasyTire(tire_radius, tire_stiffness)
core.Add(tire_rear_right)


driver = chrono.ChDriver()
driver.SetSteeringAngle(0)
driver.SetThrottle(0)
driver.SetBrake(0)
core.Add(driver)


while core.StepSimulation():
    
    core.UpdateInertiaMatrix()
    core.UpdateCollisionResponse()
    
    
    device = irrlicht.IrrlichtDevice.CreateDevice(irrlicht.VideoMode(800, 600, irrlicht.VideoMode.BPP_32))
    scene = irrlicht.Scene()
    scene.AddMesh(terrain_col_mesh)
    scene.AddMesh(terrain_vis_mesh)
    for body in core.GetBodies():
        scene.AddMesh(body.GetCollisionMesh())
        scene.AddMesh(body.GetVisualMesh())
    device.BeginScene(irrlicht.VideoBuffer.SRC_CLIENT, irrlicht.SF_NONE)
    device.DrawScene(scene)
    device.EndScene()
    device.UpdateDevice()