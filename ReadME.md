CoHAN 2.0 is an enhanced version of the Cooperative Human-Aware Navigation framework, building on the foundations of the original [CoHAN](https://github.com/sphanit/cohan_planner_multi) system [Singamaneni et al., IROS 2021](https://ieeexplore.ieee.org/document/9636613). This new iteration significantly improves the stability and robustness of the navigation pipeline in dynamic human environments. A key advancement in CoHAN 2.0 is the integration of behavior trees for context-aware mode switching, enabling more structured and scalable decision-making during navigation.

The system maintains tight integration with the ROS navigation stack and leverages the [Human-Aware Timed Elastic Band (HATEB)](https://hal.laas.fr/hal-02922029/file/Ro_Man_2020.pdf) planner for trajectory generation, while the newly introduced behavior-based layer allows the robot to seamlessly adapt between social contexts and allows easier integration of new behaviors.

In addition to behavior-based mode switching, CoHAN 2.0 incorporates reasoning about invisible humans in the environment, as introduced in [Singamaneni et al., IROS 2022](https://ieeexplore.ieee.org/document/9982186). By fusing observed human motion with static map information, the system infers the presence of humans who may be occluded or temporarily out of sensor range. This allows the planner to proactively account for hidden human activity—enhancing safety and social compliance in complex, cluttered spaces.

![](https://laas-hri.github.io/CoHAN2.0_docs/_images/cohan2.png)


If you are using this version of any of our previous versions, please cite these papers (bibtex below):

- Singamaneni, P.-T., Favier, A., & Alami, R. (2022). Watch out! There may be a Human. Addressing Invisible Humans in Social Navigation. In Proceedings of the 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 11344–11351. IEEE.

- Singamaneni, P. T., Favier, A., & Alami, R. (2021). Human-Aware Navigation Planner for Diverse Human-Robot Interaction Contexts. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS).

- Singamaneni, P. T., & Alami, R. (2020). HATEB-2: Reactive Planning and Decision Making in Human-Robot Co-navigation. In Proceedings of the International Conference on Robot & Human Interactive Communication (RO-MAN).

The documentation for this repo can be found here: [CoHAN2.0_Docs](https://laas-hri.github.io/CoHAN2.0_docs)

# Preparing things (Native)
1. Install ROS [Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu) on Ubuntu 18.04 or [Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) on Ubuntu 20.04. 
2. This installation assumes that the [ROS](http://wiki.ros.org/ROS/Installation) is already installed along with the [2D navigation stack](http://wiki.ros.org/navigation). Otherwise please install them before continuing to next steps. For ROS noetic, follow these:
3. Install the requirements
	```
	sudo apt install python-pip python-catkin-tools python-is-python3
	pip install scipy
	```
4. Clone the git repository
	```
	git clone https://github.com/sphanit/cohan2.1.git -b main ~
	cd ~/cohan2.1
	```
5. Install the dependencies using rosdep
	```
	rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
    ```

6. Follow the build instructions below.



# Using Docker
1. Clone the git repository
	```
	git clone https://github.com/sphanit/cohan2.1.git -b main ~
	cd ~/cohan2.1
    ```
2. Navigate to docker directory in the folder and build the image. For example:
    ```
    cd ~/cohan2.1/docker/noetic
    ./build-docker.sh
    ```
3. Activate the image and do the steps for building. For example:
    ```
    source ~/cohan2.1/docker/noetic/run-docker.sh
    ```


# Building Cohan and Running Examples
1. Everything will be taken care of by the script. Just run compile.sh script.
    ```
    ./compile.sh
    ```
2. Once it is built, you need to source setup.bash file to use these packages along with other ROS packages inside the system.
    ```
    source devel/setup.bash
    ```
3. Now you are set to run use it. You can run some examples as:

    ```
    roslaunch cohan_stage_navigation stage_pr2_only.launch
    ``` 
    If everything is installed correctly, you should see rviz opening and you can move the robot by giving it a goal (using 2D Nav Goal tool).

# Bibtex
```
@inproceedings{singamaneni2022watch,
title={Watch out! There may be a Human. Addressing Invisible Humans in Social Navigation},
author={Singamaneni, Phani-Teja and Favier, Anthony and Alami, Rachid},
booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
pages={11344--11351},
year={2022},
organization={IEEE}
}

@inproceedings{singamaneni2021human,
  author = {Singamaneni, Phani Teja and Favier, Anthony and Alami, Rachid},
  title = {Human-Aware Navigation Planner for Diverse Human-Robot Ineraction Contexts},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year = {2021},
}

@inproceedings{singamaneni2020hateb,
  author = {Singamaneni, Phani Teja and Alami, Rachid},
  title = {HATEB-2: Reactive Planning and Decision making in Human-Robot Co-navigation},
  booktitle = {International Conference on Robot \& Human Interactive Communication},
  year = {2020},
  doi = {10.1109/RO-MAN47096.2020.9223463}
}
```
