# Python Library for the Hiwonder 5-DOF Mobile Manipulator

This repository provides the python libraries for interfacing with the Hiwonder 5-DOF mobile manipulator. The robot platform has an onboard **Raspberry Pi 4B** which serves as the main compute unit of the system. The 5-DOF arm are driven by serial bus servos controlled over serial while the mobile base is driven by DC motors controlled by a custom driver board with communication over I2C.

Your project development will be done onboard the Raspberry Pi ideally over **SSH protocol**.


<img src = "media/hiwonder.png" width="" height="400">

## Setting up the onboard Raspberry Pi

#### Step 0: TO ADD step for connecting over SSH


#### Step 1: Create a virtual environment
- We strongly recommend that you create a new python virtual environment for all your work on the platform.
- Follow this [tutorial here](https://docs.python.org/3/tutorial/venv.html).


#### Step 2: Get this repository from Github
- I recommend you fork this repository to the account of one of your teammates and then you all can clone from the forked version.
- Follow [this tutorial](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/fork_and_clone_github_repository/Fork-and-Clone-From-GitHub.html) to understand how to fork and clone repositories


#### Step 3: Install all required Python packages
```bash
# first: make sure you have activated the virtual environment. See step 1 tutorial

# cd to the project folder
$ cd hiwonder-armpi-pro

# install all required packages from requirements.txt
$ pip install -r requirements.txt
```


### How to Run

- Before you run any script, please initialize the **pigpiod module**
``` bash
$ sudo pigpiod
```

- If setup worked well, you should be able to run the main script with the command below:
``` bash
$ sudo venv/bin/python main.py 
# this runs the main script using admin privileges and the virtual environment's python interpreter.
# N.B.: Please make sure you set the right path for your virtual environment's python interpreter above
```

### Usage Guide

<img src = "media/jstick-manual-1.png" height="300"> 
<img src = "media/jstick-manual-2.png" height="330">


