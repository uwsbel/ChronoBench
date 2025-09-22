import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono_models.robot.turtlebot as tb_models

# 1) Create the Chrono system and set gravity
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2) Create a fixed ground body with collision and visualization
ground = chrono.ChBody()
ground.SetBodyFixed(True)
# define collision shape
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(chrono.ChVectorD(5, 5, 0.1),
                                  chrono.ChVectorD(0, 0, -0.1))
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)
# visualization
vis_box = chrono.ChBoxShape()
vis_box.GetBoxGeometry().Size = chrono.ChVectorD(5, 5, 0.1)
vis_box.GetBoxGeometry().Pos = chrono.ChVectorD(0, 0, -0.1)
ground.AddVisualShape(vis_box)
system.Add(ground)

# 3) Instantiate the TurtleBot model
#    The constructor arguments are: Chrono system, contact method, vehicle fixed?
bot = tb_models.TurtleBot(system,
                          collision_type=chrono.ChMaterialSurfaceSMC(),
                          fixed=False)

# Position and orient the bot
start_pos = chrono.ChVectorD(0, 0, 0.1)
start_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # identity
bot.Initialize(start_pos, start_rot)

# 4) Create the Irrlicht application for real‐time visualization
app = chronoirr.ChIrrApp(system, "TurtleBot on Rigid Terrain",
                         chronoirr.dimension2du(1024, 768))
# add sky, lights, logo, camera
app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalLights()
# place camera behind and above the robot, looking at its origin
cam_pos = chrono.ChVectorD(1.5, -3, 1.2)
cam_target = chrono.ChVectorD(0, 0, 0.2)
app.AddTypicalCamera(cam_pos, cam_target)

app.AssetBindAll()
app.AssetUpdateAll()

# 5) Simulation loop: drive forward, then turn left, then turn right
t = 0.0
end_time = 10.0
step_size = 1e-3

while app.Run() and t < end_time:
    app.BeginScene()
    app.DrawAll()

    # simple bang‐bang steering logic
    # t < 2: straight ahead (equal wheel speeds)
    # 2 < t < 4: turn left  (left slower than right)
    # 4 < t < 6: straight
    # 6 < t < 8: turn right (left faster than right)
    # t > 8: straight
    if 2.0 < t < 4.0:
        left_speed = 1.0
        right_speed = 0.5
    elif 6.0 < t < 8.0:
        left_speed = 0.5
        right_speed = 1.0
    else:
        left_speed = right_speed = 1.0

    # Apply motor commands (rad/s)
    bot.SetMotorSpeed(left_speed, right_speed)

    # advance simulation
    app.DoStep()
    app.EndScene()
    t += step_size