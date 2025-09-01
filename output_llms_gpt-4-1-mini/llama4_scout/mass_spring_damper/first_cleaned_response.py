import chrono as ch
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


app = irr.IrrlichtApp()
app.Create IrrlichtDevice()


step_size = 0.01
end_time = 10.0


system = ch.ChSystemNSC()


ground = ch.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


mass = ch.ChBody()
mass.SetMass(1.0)
mass.SetPos(ch.ChVectorD(0, 1, 0))
system.Add(mass)


mass_shape = ch.ChSphereShape(0.2)
mass.AddVisualShape(mass_shape)


spring_damper = ch.ChLinkTSDA()
spring_damper.SetPoints(ch.ChVectorD(0, 0, 0), ch.ChVectorD(0, 1, 0))
spring_damper.SetRestLength(1.0)
spring_damper.SetSpringConstant(10.0)  
spring_damper.SetDampingCoefficient(2.0)  
system.Add(spring_damper)


spring_damper.Connect(ground, mass)


system.SetGravitationalAcceleration(ch.ChVectorD(0, -9.81, 0))


system.SetIntegrator(ch.ChTimestepperEulerImplicitLinear)


ground_shape = ch.ChBoxShape(10, 0.1, 10)
ground.AddVisualShape(ground_shape)


app.AddCamera(ch.ChVectorD(0, 2, 3), ch.ChVectorD(0, 0, 0))
app.SetLightPosition(ch.ChVectorD(5, 5, 5))
app.ShowInfos(True)


while app.Run(step_size):
    system.DoStepDynamics(step_size)

import chrono as ch
import chrono.irrlicht as irr

def main():
    
    ch.ChEngine.Initialize()

    
    app = irr.IrrlichtApp()
    app.CreateIrrlichtDevice()

    
    step_size = 0.01
    end_time = 10.0

    
    system = ch.ChSystemNSC()

    
    ground = ch.ChBody()
    ground.SetBodyFixed(True)
    system.Add(ground)

    
    mass = ch.ChBody()
    mass.SetMass(1.0)
    mass.SetPos(ch.ChVectorD(0, 1, 0))
    system.Add(mass)

    
    mass_shape = ch.ChSphereShape(0.2)
    mass.AddVisualShape(mass_shape)

    
    spring_damper = ch.ChLinkTSDA()
    spring_damper.SetPoints(ch.ChVectorD(0, 0, 0), ch.ChVectorD(0, 1, 0))
    spring_damper.SetRestLength(1.0)
    spring_damper.SetSpringConstant(10.0)  
    spring_damper.SetDampingCoefficient(2.0)  
    system.Add(spring_damper)

    
    spring_damper.Connect(ground, mass)

    
    system.SetGravitationalAcceleration(ch.ChVectorD(0, -9.81, 0))

    
    system.SetIntegrator(ch.ChTimestepperEulerImplicitLinear)

    
    ground_shape = ch.ChBoxShape(10, 0.1, 10)
    ground.AddVisualShape(ground_shape)

    
    app.AddCamera(ch.ChVectorD(0, 2, 3), ch.ChVectorD(0, 0, 0))
    app.SetLightPosition(ch.ChVectorD(5, 5, 5))
    app.ShowInfos(True)

    
    while app.Run(step_size):
        system.DoStepDynamics(step_size)

if __name__ == "__main__":
    main()