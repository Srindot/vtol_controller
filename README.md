# Designing a controller for Vertical Take Off and Landing (VTOL) Vehicle

This repository contains switching PID controller for transitioning phase for a vtol vehicle.


## *Simulation Stack*
- ROS2
- Gazebo
- PX4
- QGroundController (Ground Station)

## *Simulated Vehicle*
The PID switching controller was developed and fine tuned for the vtol vehicle in the gazebo sim.

and the transition phase was studied.

![alt text](image.png)

## *Setup*
### **Prerequisites** 
Make sure to setup 
    - Docker 
    - Vscode with docker and devcontainer extensions

if you face any issue with nvidia card visit [this](https://github.com/Srindot/Waypoint_Tracking_in_ROS-PX4.git) repo for more information.


### **Usage** 
1. Clone this repository 
    ```bash 
    git clone https://github.com/Srindot/vtol_controller.git
    ```

2. Open the project with vscode and make sure you install docker and devcontainer extensions.


3. Press `ctrl` + `SHIFT`+ `P` to open up the command pallete, then search for `Dev Containers: Rebuild and Reopen in Container` and select it. 
    > Note: This will take a long time, so no need to fret.

4. After entering container, press `ctrl` + `SHIFT`+ `P` again, then search for `Tasks: Run Tasks` and then run the sessions sequentially.


## *Issues*
If you face any troubles, just raise a issue. 