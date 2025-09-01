importchrono
import chronoirrlicht as chrono_irr
import numpy as np


system = chrono.ChSystem()
system.SetGravity(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetMass(0)  
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground_shape = chrono.ChBoxShape()
ground_shape.GetDimensions().x = 20
ground_shape.GetDimensions().y = 20
ground_shape.GetDimensions().z = 1
ground.AddVisualShape(ground_shape)
system.Add(ground)


bot = chrono.ChTurtlebot()
bot.SetPos(chrono.ChVectorD(0, 0.5, 0))  
bot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  
system.Add(bot)


app = chrono_irr.ChIrrApp(system, "Turtlebot Simulation", chrono_irr.dimension(800, 600))
app.AddTypicalIrrlichtLights()
app.AddTypicalIrrlichtCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))


timestep = 0.01
time = 0
max_time = 5  


motors = bot.GetMotors()
left_speed = 1  
right_speed = 1  


while app.GetDevice().run() and time < max_time:
    
    if time < 1:
        
        motors[0].SetSpeed(left_speed)
        motors[1].SetSpeed(right_speed)
    elif 1 <= time < 2:
        
        motors[0].SetSpeed(-left_speed)
        motors[1].SetSpeed(right_speed)
    elif 2 <= time < 3:
        
        motors[0].SetSpeed(left_speed)
        motors[1].SetSpeed(-right_speed)
    else:
        
        motors[0].SetSpeed(0)
        motors[1].SetSpeed(0)
    
    
    system.DoStepDynamics(timestep)
    
    
    app.Render()
    
    
    time += timestep

print("Simulation completed!")