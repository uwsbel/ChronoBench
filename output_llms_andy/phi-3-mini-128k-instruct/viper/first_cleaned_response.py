from pychoro import *

import irrlicht as irr




myChrono = ChronoSystem()




myChrono.AddGravity(Vector3D(0,-9.81,0))




myGround = createGround()

myChrono.AddBody(myGround)




myViper = createViper()

myChrono.AddBody(myViper)




myDriver = createDriver(myViper)

myChrono.AddDriver(myDriver)




steering_params = {

    'initial_steering': 0,

    'final_steering': 90,

    'duration': 5.0  

}




myApplication = Application(application_display_config_file="default.cfg")

myApplication.setWindowTitle("Viper Rover Simulation")




myApplication.addCamera("main", irr.vector3df(4.0, 3.0, 2.0), irr.vector3df(0, 0, 0), 10.0, 90)

myApplication.addLight("main", irr.vector3df(5.0, 5.0, 5.0), irr.vector3df(0, 1, 0))




while myApplication.application_is_running():

    myApplication.startTimer()

    myChrono.Step(1.0/60.0, 10)  


    

    elapsed_time = myApplication.getTimer()

    new_steering = steering_params['initial_steering'] + (steering_params['final_steering'] - steering_params['initial_steering']) * (elapsed_time / steering_params['duration'])

    myDriver.setSteering(new_steering)


    

    myApplication.drawAll()

    myApplication.update()




myApplication.closeApplication()