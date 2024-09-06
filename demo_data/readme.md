This folder contains five major categories of simulations, encompassing 34 unique dynamical systems, with a total of 102 simulations (3 variations per system).

### Categories and Systems:

- **Multibody Systems (MBS)**:
  - `["gear", "mass_spring_damper", "particles", "pendulum", "slider_crank"]`
  
- **Finite Element Analysis (FEA) Systems**:
  - `["beam", "buckling", "cable", "rotor", "tablecloth"]`

- **Sensor Systems (SEN)**:
  - `["camera", "gps_imu", "lidar", "veh_app"]`

- **Robotic Systems (RBT)**:
  - `["curiosity", "handler", "viper", "turtlebot", "vehros", "sensros"]`

- **Vehicle Systems (VEH)**:
  - `["art", "citybus", "feda", "gator", "hmmwv", "kraz", "m113", "man", "rigid_highway", "rigid_multipatches", "scm", "scm_hill", "sedan", "uazbus"]`

### SimBench Overview:

SimBench provides a diverse range of simulation scenarios across five key application areas: multibody dynamics (MBS), vehicle dynamics (VEH), robotics (RBT), finite element analysis (FEA), and sensor integration (SEN). These scenarios are designed to evaluate specific aspects of S-LLM performance, such as Digital Twin (DT) setup, problem reasoning, and model adjustments in PyChrono.

While the primary focus is on benchmarking the J-LLM methodology, the J-LLM in Chrono SimBench supports a wide array of multi-physics simulations, covering the following areas:

- **Collision, Contact, and Friction Dynamics (MBS)**: Scenarios involving typical mechanisms such as multi-link arms, gear systems, and slider-crank setups test the S-LLM’s ability to handle complex mechanical interactions.

- **Vibration, Deformation, Stress, and Strain (FEA)**: This category includes simulations of cables, beams, shells, and plates, evaluating the S-LLM’s proficiency in structural and stress analysis.

- **Vehicle Dynamics (VEH)**: Realistic driving scenarios involving city buses, off-road vehicles (e.g., HMMWV, M113), trucks (e.g., Kraz, MAN), and sedans test the S-LLM’s ability to simulate vehicles. These simulations incorporate models for drivers, engines, transmissions, and tires, along with high-level control policies integrated with various sensors.

- **Sensor Integration (SEN)**: Perception tasks involving GPS, IMU, LiDAR, and camera sensors are used to assess the S-LLM’s ability to support autonomous vehicles and robotics systems.

- **Robotic Dynamics (RBT)**: Robotics scenarios featuring systems like Turtlebot, Curiosity, and VIPER are included, alongside simulations of granular dynamics and deformable terrain, such as the Soil Contact Model (SCM), which is crucial for off-road operations involving robots and vehicles.

### Simulation Tasks:

SimBench features 102 demonstration tasks across 34 unique physical systems from the five categories listed above. Each task is structured into three stages of increasing complexity, designed by simulation experts to challenge the S-LLM’s capabilities in setting up and modifying Digital Twins. These tasks provide a robust evaluation of the S-LLM's performance across different simulation environments.
