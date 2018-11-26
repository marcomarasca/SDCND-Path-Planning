# Highway Path Planning
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[pp_architecture]: ./images/path_planning.png "Path planning architecture"
[pp_frenet]: ./images/frenet.png "Frenet coordinates"
[pp_frenet_2]: ./images/frenet_2.png "Frenet vs cartesian coordinates"
[pp_gif]: ./images/path_planning.gif "Path Planning on an Highway"
[pp_stuck_gif]: ./images/path_planning_stuck.gif "Path Planning Limitations"

![Gif: Path Planning on an Highway][pp_gif]

Overview
---

This repository contains a C++ implementation of a *Path Planning* module used to direct the controller of a vehicle to move around a fixed map of waypoints, simulating a car driving on an highway environment. The localization data about the ego vehicle as well as the data about surrounding traffic comes from the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim) that feeds it to the application using [WebSockets](https://en.wikipedia.org/wiki/WebSocket) messages. The [main](./src/main.cpp) file processes the incoming messages, parsing the data that is then processed by the [Path Planner](./src/path_planner.cpp) class.

The goal of the project is to safely navigate around the virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided by the simulator, and a sparse map list of waypoints around the highway is detailed in the [provided csv file](./src/data/highway_map.csv). The program generates a trajectory in global coordinates that is sent to the simulator, which in turn will refresh the car position every 0.02s (e.g. using a perfect controller) according to the given trajectory (which is sent over websocket messages) trying to go as close as possible to the 50 MPH speed limit, passing slower traffic when possible. The planner should generate trajectories that avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. Additional constraints are that the car should not experience a total *acceleration* over 10 m/s^2 and *jerk* that is greater than 10 m/s^3.

The car in the simulator uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration.

### Highway Map

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to **6945.554**.

### Message Protocol

The data received in each websocket message contains the ego car localization data, the previous path information and the sensor fusion data.

#### Localization Data

* ["x"] The car's x position in map coordinates
* ["y"] The car's y position in map coordinates
* ["s"] The car's s position in frenet coordinates
* ["d"] The car's d position in frenet coordinates
* ["yaw"] The car's yaw angle in the map
* ["speed"] The car's speed in **MPH**

#### Previous Path

Note: Return the previous list but with **processed points removed**, can be a nice tool to show how far along
the path has processed since last time. 

* ["previous_path_x"] The previous list of x points previously given to the simulator
* ["previous_path_y"] The previous list of y points previously given to the simulator
* ["end_path_s"] The previous list's last point's frenet *s* value
* ["end_path_d"] The previous list's last point's frenet *d* value

#### Sensor Fusion Data

* ["sensor_fusion"] A 2d vector of cars, each element contains:
    * [0] car's unique ID
    * [1] car's x position in map coordinates
    * [2] car's y position in map coordinates
    * [3] car's x velocity in **m/s**
    * [4] car's y velocity in **m/s**
    * [5] car's *s* position in frenet coordinates
    * [6] car's *d* position in frenet coordinates

Path Planner
---
In autonomous driving the *Path Planner* is in charge of generating *safe* and *efficient* trajectories that are fed to the agent controller (in one form or the other, depending on the chosen protocol) in order to navigate to a certain goal. The idea is that the data computed from other modules such as the localization module and the sensor fusion module are processed in order to generate a set of potential trajectories that are evaluated in order to find the one that minimize a certain (set of) cost functions that leads to the most efficient as well as safe movement towards the given goal.

![Path planning components][pp_architecture]

Path planning can be a considerably complex module, and probably one of the most difficult parts to design and implement in an autonomous agent, roughly we can split it into 3 main components:

* **Prediction**: Using the sensor fusion data generates models to predict the movement of the moving objects surrounding the main agent
* **Behavior**: Taking the information about the current agent and the surrounding environment as well as the current goal generates a plan for the agent, depending on the various aspects of the current state (e.g. which environment, predictions about the other moving objects, distance to the goal etc)
* **Trajectory**: Simply put, the goal is to generate an efficient trajectory that satisfies a certain set of constraints (e.g. speed, acceleration, jerk, position etc.)

The 3 components work together to decide how a vehicle should move around the environment.

Implementation Details
---

For the implementation in this repository we can make a set of assumptions given the constraints of the project, in particular we know in advance the type of environment (**highway**) and we can therefore simplify our workflow while still maintaining a sufficient level of efficiency and safety. The vehicle drives continuously around a closed track, therefore the *goal* is simply to keep driving forward at the maximum speed reachable according to the surrounding traffic. Even though we are supplied with a map with subsequent endpoints to follow the map is circular, therefore the next waypoint is always ahead. In reality the goal should be set according to the next waypoint and cost functions should be designed around that information in order to select the best plan.

Additionally the *prediction model* used in this implementation uses a simple constant speed and acceleration model assuming that the surrounding vehicles will not change behavior often and will mostly maintain a constant speed. This still manages to prevent collisions even when the other vehicles suddenly change lanes or break in front of the ego vehicle given the high refresh rate of the path planner that can quickly adapt to changes.

Finally the *behavior planner* uses a simplified state machine which works around 3 logical states: *keep current lane*, *change lane to left* and *change lane to right*. The behavior planner uses the trajectory generator as well as a cost function based path evaluator to decide the next best target plan producing directly the best trajectory to follow.

## Code Structure

The code is split in modules according to the responsibility:

* **[main.cpp](./src/main.cpp)**: Main entry-point of the program, process the incoming websocket messages redirecting the processing to the [Path Planner](./src/path_planner.cpp).
* **[map.cpp](./src/map.cpp)**: Utility class to handle the vehicle localization around the map and to handle the transformation between cartesian and frenet coordinates. Uses a [spline](./src/spline.h) to improve the accuracy of the evaluation of the position of the vehicle in frenet space.
* **[utils.h](./src/utils.h)**: Utility header that contains various functions and structures used throughout the application.
* **[vehicle.cpp](./src/vehicle.cpp)**: Represents a vehicle on the road along with its Frenet state and trajectory (in frenet space), contains methods to predict states according to the vehicle dynamics and states.
* **[path_planner.cpp](./src/path_planner.cpp)**: Main orchestrator that directs the responsibilities to the various modules, takes care of parsing the telemetry received by the simulator as well as accounting for [inconsistencies](./src/path_planner.cpp#104) in the data received from the simulator.
* **[trajectory_generator.cpp](./src/trajectory_generator.cpp)**: Takes care of generating trajectories for the various states, using either a jerk minimizing trajectory generation or a supplied prediction function when the target state is not known.
* **[behaviour_planner.cpp](./src/behvaiour_planner.cpp)**: Takes in input the ego vehicle and the information about the traffic and evaluate the current state of the environment to generate an optimal plan. Uses the trajectory generator to create potential plans and a trajectory evaluator to decide which trajectory is the best to follow.
* **[trajectory_evaluator.cpp](./src/trajectory_evaluator.cpp)**: Takes in input a plan generated by the behavior planner and the current traffic on the road and using a set of cost functions provides a cost for the given plan.
* **[cost_functions](./src/cost_functions.cpp)**: Implementation of the cost functions used by the trajectory evaluator.

## Frenet Coordinates

The project mainly works around [frenet coordinates](https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas#Kinematics_of_the_frame) instead of the traditional cartesian coordinates. In brief the idea of Using Frenet coordinates is to have a system aligned with the direction of the road along its longitudinal axes (**s** component) and displacement (**d** component):

![Frenet Coordinates][pp_frenet]

This allows to have a coordinate system that is allows a simple representation of the movement of the car within a road and lanes boundaries:

![Frenet Coordinates][pp_frenet_2]

In this implementation the *s* and *d* components are split and maintained separately for each [vehicle](./src/vehicle.h), and a [trajectory](./src/utils.h#43) is represents as a list of frenet states that change with time.

## Trajectory Generation

To drive around an highway an effective method to generate efficient and low *jerk* trajectories is to use a *jerk* minimizing trajectory generation method, with the *jerk* defined as the change in acceleration over time. The idea is that reducing the jerk of a trajectory maximizes the comfort of the the passengers of the vehicle. The [implementation](./src/trajectory_generator.cpp) in this repository follows the approached presented in the paper [Optimal trajectory generation for dynamic street scenarios in a Frenét Frame](https://www.semanticscholar.org/paper/Optimal-trajectory-generation-for-dynamic-street-in-Werling-Ziegler/f95577df76d3a78de71ff0ced34140708bc3e2b4) using a *quintic polynomials* to find the coefficients that reduces the jerk along a trajectory that is generated from a given starting *state* and target *state*, where each state contains the *position*, *velocity* and *acceleration*. Given that we split the *s* and *d* components we generate two different set of coefficients that represents the longitudinal and later displacement separately. Note that while this method effectively translates into a correct trajectory, the constraints are put only on the starting and ending state therefore what happens in the middle may violate the overall constrains (e.g. speed, acceleration), the implementation here tries to limit this effect while a trajectory is generated putting the constraints into effect when generating a new step of the trajectory.

## Behavior Planner

The approach taken in this project uses simple approach where according to the current state and for each available lane a [new plan is generated](./src/behaviour_planner.cpp#131) taking into account the lane traffic and speed. The new *s* and *d* components are chosen according to the vehicles around the ego vehicle, computing a safe distance for the current velocity (e.g. the space taken to come to a stop) and the predicted position at time *t*. The generated target position and speed are used to generate a *potential trajectory*, taking into account the *average delay* due to processing time, the idea is to keep a part of the previous trajectory considering that while the program is computing the next trajectory the simulator is consuming other points of the past trajectory. 

The final trajectory is therefore generated starting at a "future" state according to the average delay (See [behaviour_planner.cpp, line#146](./src/behaviour_planner.cpp#146)).

Once the potential trajectory is generated the plan is fed to the [Plan Evaluator](./src/plan_evaluator.cpp) that simply computes the information along the given trajectory (such as collision and lane speed) with the predictions of the traffic and outputs a total cost according to various factors given by the following cost functions:

## Cost Functions

The [Plan Evaluator](./src/plan_evaluator.cpp) takes advantage of a set of independent [cost functions](./src/cost_functions.cpp), in particular for this project the following cost functions are implemented:

* [Collision](./src/cost_functions.cpp#9): Taking into account the predictions of the cars on the road assigns a binary value if a collision is detected along a trajectory (including from behind)
* [Distance buffer](./src/cost_functions.cpp#24): Similar to the previous cost function, computes a value in the range [0, 1] according to the minimum distance from each vehicle in the trajectory
* [Average speed](./src/cost_functions.cpp#34): Computes a value in the range [0, 1] according to the average speed within the trajectory
* [Lane speed](./src/cost_functions.cpp#48): Computes a value in the range [0, 1] according to the average speed of the target lane of the trajectory (considering only the vehicles that are ahead of the ego vehicle)
* [Traffic](./src/cost_functions.cpp#64): Computes a value in the range [0, 1] that takes into account the number of vehicles ahead in the target lane of the trajectory, also consider the minimum distance from the vehicles ahead (e.g. if the traffic is far then the cost is lower)
* [Change plan](./src/cost_functions.cpp#83): Considers the difference between the current plan lane and the target lane of the trajectory, basically giving a cost to switching lanes.
* [Unfinished plan](./src/cost_functions.cpp#99): Similar to the previous one but considers the distance to the current plan lane, this acts as a deterrent when a lane change is already in progress to avoid swerving in the middle of the manoeuvre.

Each cost is multiplied by a weight and summed up to define the final cost of a plan.

## Limitations

The current implementation makes a set of assumptions about the overall pipeline that introduce various limitations, in particular the behaviour planner only considers lanes that are adjacent to the ego vehicle. This may limit the planner in considering more efficient scenarios, for example in adapting the speed in order to move slowly toward a faster lane that is more than 1 lane away:

![Gif: Path Planning Stuck][pp_stuck_gif]

In the image above the planner initially chose the leftmost lane because the overall cost of the inner lanes is higher due to speed limitations (in general the left lanes are used for passing cars). Later on instead of slowing down and leading to the rightmost lane in order to pass the slower traffic ahead, the planner simply remains stuck to the choice between the two inner lanes on the left, following the vehicles ahead at lower speed. Ideally in such a scenario the planner should have detected the higher speed of the rightmost lane and planned for a longer period of time. 

In the current implementation the behavior planner is updated at each tick together with the optimal trajectory, this may sometimes lead to situations in which the vehicle switches lanes, just to go back to the original lane (e.g. a car from behind is approaching faster and may increase the risk of collisions). A better approach would be to loosen the timing on the behavior update in respect to the trajectory generation towards the set goal. While the implementation may seem robust enough to avoid collisions in such situations, the overall comfort level is lower due to the number of lane changes.

Additionally the cost functions implemented in this project, even though effective, are relatively simple and they do not actually consider the distance to the goal simply because the map is a circuit.

Getting Started
---

In order to run the program you need the simulator for this specific environment provided by [Udacity](https://www.udacity.com/) which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even better [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. The version compatible with the simulator is the uWebSocketIO branch **e94b6e1**.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing as follows from the project root folder:

1. ```mkdir build```
2. ```cd build```
3. ```cmake .. && make```
4. ```./path_planning```

Note that to compile the program with debug symbols you can supply the appropriate flag to cmake: ```cmake -DCMAKE_BUILD_TYPE=Debug .. && make```.

The application takes in input 3 optional parameters:

- **-f, --filename** `<path>`: The path to the file containing the map waypoints, defaults to ../data/highway_map.csv
- **-d, --debug**: If the flag is present debugging information (such as the costs for each function for the generated paths) are send to the standard output
- **-g, --graphics**: If the flag is present instead of loggin messages a ascii representation of the road with the detected vehicle and the current trajectory is drawn to the screen

Once the application is running and the websocket server is listening the Udacity simulator can be started that will connect to the websocket server starting sending messages with the current position as well as the sensor fusion data.

![Gif: Path Planning on an Highway][run_gif]

#### Other Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

Environment Setup
---

This project was developed under windows using the windows subsystem for linux ([WSL](https://docs.microsoft.com/en-us/windows/wsl/install-win10)) with Ubuntu Bash 16.04 together with [Visual Studio Code](https://code.visualstudio.com/).

The steps to setup the environment under mac, linux or windows (WSL) are more or less the same:

- Review the above dependencies
- Clone the repo and run the appropriate script (./install-ubuntu.sh under WSL and linux and ./install-mac.sh under mac), this should install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) from the branch **e94b6e1**

Under windows (WSL) and linux you can make a clean installation as follows:

1. ```sudo apt-get update```
2. ```sudo apt-get install git```
3. ```sudo apt-get install cmake```
4. ```sudo apt-get install openssl```
5. ```sudo apt-get install libssl-dev```
6. ```git clone https://github.com/Az4z3l/CarND-Path-Planning```
7. ```sudo rm /usr/lib/libuWS.so```
8. ```./install-ubuntu.sh```

#### Debugging with VS Code

Since I developed this project using WSL and Visual Studio Code it was very useful for me to setup a debugging pipeline. VS Code comes with a official Microsoft cpp extension that can be downloaded directly from the marketplace: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools. After the installation there are a few things to setup in order to make it work with the subsystem for linux, personally I went with the default Ubuntu distribution.

For the following setup I assume that the repository was cloned in **D:/Dev/CarND-Path-Planning/**.

##### Setup the language server (for IntelliSense)

From the official documentation [https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/Windows%20Subsystem%20for%20Linux.md](https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/Windows%20Subsystem%20for%20Linux.md): 

Simply Crtl+P and select "C/Cpp: Edit Configurations", this will create a c_cpp_properties.json file that can be configured as follows:

```json
{
    "configurations": [
        {
            "name": "WSL",
            "intelliSenseMode": "gcc-x64",
            "compilerPath": "/usr/bin/gcc",
            "includePath": [
                "${workspaceFolder}/**"
            ],
            "browse": {
                "path": [
                    "${workspaceFolder}"
                ],
                "limitSymbolsToIncludedHeaders": true
            },
            "defines": [
                "_DEBUG",
                "UNICODE",
                "_UNICODE"
            ],
            "cStandard": "c11",
            "cppStandard": "c++17"
        }
    ],
    "version": 4
}
```

##### Setup the Debugger

From the official documentation [https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/Debugger/gdb/Windows%20Subsystem%20for%20Linux.md](https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/Debugger/gdb/Windows%20Subsystem%20for%20Linux.md):

First install gdb in the WSL:

```
sudo apt install gdb
```

Then simply create a lunch configuration from VS Code: "Debug" -> "Add Configuration.." and setup the launch.json as follows:

```json
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "/mnt/d/Dev/udacity/term3/CarND-Path-Planning/build/path_planning",
            "args": ["../data/highway_map.csv", "-fThreading"],
            "stopAtEntry": false,
            "cwd": "/mnt/d/Dev/udacity/term3/CarND-Path-Planning/build/",
            "environment": [],
            "externalConsole": true,
            "windows": {
                "MIMode": "gdb",
                "setupCommands": [
                    {
                        "description": "Enable pretty-printing for gdb",
                        "text": "-enable-pretty-printing",
                        "ignoreFailures": true
                    }
                ]    
            },
            "pipeTransport": {
                "pipeCwd": "",
                "pipeProgram": "c:\\windows\\sysnative\\bash.exe",
                "pipeArgs": ["-c"],
                "debuggerPath": "/usr/bin/gdb"
            },
            "sourceFileMap": {
                "/mnt/d": "D:\\"
            }
        }
    ]
}
```

Note how the program is mapped directly into the file system of the WSL and piped through bash.exe (the paths are relative to the WSL environment).

Now you are ready to debug the application directly from VS Code, simply compile the application from within the WSL with the debug symbols:

```cmake -DCMAKE_BUILD_TYPE=Debug .. && make```

And run the debugger from VS Code (e.g. F5) :)

