# ChronoBench benchmark (`demo_data/`)

This folder is the benchmark itself: five major categories of simulation scenarios (setups, themes), encompassing 34 mechatronic systems, for a total of 102 simulations (3 variations per system). The hope is that this number steadily goes up to improve the breadth and depth of the simulation setups available. Per-system files and their roles are indexed in `demo_data/manifest.json`; see also the "add a system" guide below.

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

### ChronoBench Overview:

ChronoBench provides a diverse range of simulation scenarios across five key application areas: multibody dynamics (MBS), vehicle dynamics (VEH), robotics (RBT), finite element analysis (FEA), and sensor integration (SEN). These scenarios are designed to evaluate specific aspects of S-LLM performance, such as virtual experiment script setup, problem reasoning, and the ability to carry out model adjustments.

A brief discussion of the classes of simulations used in this benchmark are as follows:

- **Collision, Contact, and Friction Dynamics (MBS)**: Scenarios involving typical mechanisms such as multi-link arms, gear systems, and slider-crank setups test the S-LLM's ability to handle complex mechanical interactions.

- **Vibration, Deformation, Stress, and Strain (FEA)**: This category includes simulations of cables, beams, shells, and plates, evaluating the S-LLM's proficiency in structural analysis.

- **Vehicle Dynamics (VEH)**: Realistic driving scenarios involving city buses, off-road vehicles (e.g., HMMWV, M113), trucks (e.g., Kraz, MAN), and sedans test the S-LLM's ability to simulate vehicles. These simulations incorporate models for drivers, engines, transmissions, and tires, along with high-level control policies integrated with various sensors.

- **Sensor Integration (SEN)**: Perception tasks involving GPS, IMU, LiDAR, and camera sensors are used to assess the S-LLM's ability to support autonomous vehicles and robotics systems.

- **Robotic Dynamics (RBT)**: Robotics scenarios featuring systems like Turtlebot, Curiosity, and VIPER are included, alongside simulations of deformable terrain, such as the Soil Contact Model (SCM), which is crucial for off-road operations involving robots and vehicles.

### Simulation Tasks:

ChronoBench features 102 demonstration tasks across 34 unique physical systems from the five categories listed above. Each task is structured into three stages of increasing complexity, designed by simulation experts to challenge the S-LLM's capabilities in setting up and modifying virtual experiment scripts. These tasks provide a robust evaluation of the S-LLM's performance across different simulation environments.

## Adding a system

The benchmark is frozen for the published contract (`v1.0-ieee-access-2026`). Adding a system
changes the `demo_data` content hash, so it **breaks comparability** with that contract and belongs
to a NEW contract version (see `contracts/HOW_TO_VERSION.md`). To add a system `<name>`:

1. Create `demo_data/<name>/` with the per-turn files (the layout every system follows):
   `input{1,2,3}.txt` (the prompts), `truth{1,2,3}.py` (expert reference scripts),
   `pyinput{2,3}.py` (the starter code handed to the agent on turns 2-3), and `output{1,2,3}.json`
   (Alpaca-style records). Generate the comment-stripped `cleaned_truth{1,2,3}.py` with
   `python scoring/clean_truth.py`.
2. Add `<name>` to the correct category in `chronobench/systems.py` (`CATEGORIES`).
3. Regenerate the index: `python scoring/generate_manifest.py`.
4. Cut a new contract version (re-pin the `demo_data` hash) per `contracts/HOW_TO_VERSION.md`.

