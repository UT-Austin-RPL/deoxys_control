<p align="center">
<img src="./shortened_deoxys_title.png">
</p>

# Deoxys
[**[Documentation]**](https://ut-austin-rpl.github.io/deoxys-docs/html) &ensp; 

We will change the name when all the changes are complete.

This repository is designed for light-weight control of Franka Emika Panda arm, targetting at various robot learning projects.

[Currently the infrastructure will focus on Franka Emika Panda
robots. This might change in the future.]

We choose zmqpp for C++ wrapper of zmq because it can provide a
non-blocking api, which is important for communication with robot.

## Citing
If you use this codebase for your research projects, please cite our codebase based on the following project:

```
@article{zhu2022viola,
  title={VIOLA: Imitation Learning for Vision-Based Manipulation with Object Proposal Priors},
  author={Zhu, Yifeng and Joshi, Abhishek and Stone, Peter and Zhu, Yuke},
  journal={arXiv preprint arXiv:2210.11339},
  year={2022}
}
```


# Installation of codebase

Overall, the installation has three parts:
1. Install dependencies by running `InstallPackage`
2. Compile desktop-side codebase (Python)
3. Compile NUC-side codebase (C++)

Here are the details. For more information, please refer to the [Codebase Installation Page](https://ut-austin-rpl.github.io/deoxys-docs/html/installation/codebase_installation.html).

Clone this repo to the robot workspace directory on Desktop computer (e.g. `/home/USERNAME/robot-control-ws`)

``` shell
cd deoxys_control/deoxys
```

## Install dependencies

Run the `InstallPackage` file to install necessary packages.
``` shell
./InstallPackage
```


## Deoxys - Desktop

Make sure that you are in your python virtual environment before
	building this.
``` shell
make -j build_deoxys=1
```

And install all the python dependencies (feel free to add pull requests if anything is missing) from `deoxys_control/requirements.txt`, by doing:
```shell
pip install -U -r requirements.txt
```

Q: We might want to read states faster than the control rate. The best
way might be run another node and write states into redis buffer?


## Franka Interface - Intel NUC

Franka Interface is the part which is supposed to run on NUC. Run this 
command in directory `deoxys_control/deoxys/` on Intel NUC. 

``` shell
make -j build_franka=1
```

## A laundry list of pointers:
   - [How to turn on/off the robot](https://ut-austin-rpl.github.io/deoxys-docs/html/tutorials/running_robots.html)
   - [How to install spacemouse](https://ut-austin-rpl.github.io/deoxys-docs/html/tutorials/using_teleoperation_devices.html)
   - [How to set up the RTOS](https://ut-austin-rpl.github.io/deoxys-docs/html/installation/system_prerequisite.html)
   - [How to record and replay a trajectory](https://ut-austin-rpl.github.io/deoxys-docs/html/tutorials/record_and_replay.html)
   - [How to write a simple motor program](https://ut-austin-rpl.github.io/deoxys-docs/html/tutorials/handcrafting_motor_program.html)

# Control the robot

## Commands on Desktop

Here is a quick guide to run `Deoxys`.

Under `deoxys_control/deoxys`,  run
``` shell
python scripts/run_deoxys_with_spacemouse.py 
```
Change 1) spacemouse vendor_id and product_id ([here](https://github.com/UT-Austin-RPL/robot_infra/blob/74eb784013fb713a0c535dd0e983d5a102008e33/deoxys/scripts/run_deoxys_with_spacemouse.py#L14)) 2) robot interface 
config ([here](https://github.com/UT-Austin-RPL/robot_infra/blob/74eb784013fb713a0c535dd0e983d5a102008e33/deoxys/scripts/run_deoxys_with_spacemouse.py#L17)) if necessary.

You might also check and change the PC / NUC names [here](https://github.com/UT-Austin-RPL/robot_infra/blob/master/deoxys/config/charmander.yml). 

## Commands on Control PC (Intel NUC)

Under `deoxys_control/deoxys`, run two commands. One for real-time control of the arm, one for non
real-time control of the gripper.

``` shell
bin/franka-interface config/charmander.yml
```

``` shell
bin/gripper-interface config/charmander.yml
```

