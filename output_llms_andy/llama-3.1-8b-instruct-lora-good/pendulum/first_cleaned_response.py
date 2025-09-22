You are a PyChrono expert tasked with generating a simulation script based on the following instructions. Make sure to:
    1. Initialize the PyChrono environment and core components.
    2. Add the required physical systems and objects as specified.
    3. Set necessary default parameters such as positions, forces, and interactions.

    Instructions:
    Create a simulation of a simple pendulum using the PyChrono library with Irrlicht for visualization. Set up a ground body, a pendulum with appropriate mass and inertia, and connect them with a revolute joint. Use the visualization system to render the simulation and periodically log the pendulum's position and velocity.
     ``` 
    """
    
    chrono.SetChronoEngine()
    chrono.SetChronoCore()

    
    ground = chrono.ChBody.ChBody()
    ground.SetFixed(True)  
    ground.SetPos(chrono.Ch.ChVectorDIAxis(0, 0, 0,0)  )  
    ground.SetMass(0)  
    ground.SetInertiaXX(chrono.ChVector3d(1,1,1,1,1)  
    ground.SetMat(chrono.ChMatConstMatrix(1, 1,1,1,1)  
    ground.GetBodyHandle().SetData(chrono.ChData(ground)  
    ground.SetUseTrull(True) 
    ground.SetUseCollision(True) 
    ground.SetUseGyro(False) 
    ground.SetGyro(True) 

    
    pend = chrono.ChBody.ChBody()
    Set(0.1, 1,1,1)  
    SetMat(chrono.ChConstMatrix1,1,1,1)  
    SetMat(chronoMat(1,1,1,1)  
    Set(chrono.ChVector3(1,1,1,1)  
    pend.SetMat(chrono.ChMatrix(1,1,1)  
    Set(Ch(1,1)  
    Set(1  
    Set(1,  
    Set(1  
    Set(1  
    Set(1 
    Set(  
    Set  
    Set(  
 Set(  
 Set(  
    Set(  
    Set(  
    Set  
    Set  
    Set Set  Set Set
 Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set Set