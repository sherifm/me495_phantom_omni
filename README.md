ME495 Setup Instructions
========================


## Trep installation ##

If you want to actually run the demo, you'll need to install [trep]. Trep is a
piece of software for simulating the dynamics of rigid bodies (ME314!), and for
performing optimal control calculations (ME454!) that was originally developed
by [Elliot Johnson][ejo]. Trep is a Python module that is hosted on the [Python
Package Index][pypi], and is installable using [pip] or [easy\_install][easy].

Follow the [trep install instructions][tinstall] to install trep. I recommend
following the steps in section 1 and sub-section 2.1. Note you must have `pip`
installed for sub-section 2.1.

## Obtaining a copy of the repository ##

I've created a special *branch* in my `git` repository for the class to look
at. In this section I'll tell you how to both get a copy of the code, and ensure
you are on the right branch. First open a terminal, and navigate to the `src/`
directory of your workspace

```bash
cd ~/catkin_ws/src
```

Then you will *clone* the repository into that directory:

```bash
git clone https://github.com/jarvisschultz/trep_puppet_demo.git -b me495
```

Then running

```bash
cd trep_puppet_demo
git branch
```

should return `* me495` indicating you are on the `me495` branch. You should
also now see all of the files for this package located in
`~/catkin_ws/src/trep_puppet_demo/`.

This package has no custom messages or services, but it does have one piece of
C++ code. Therefore we must build this package. To accomplish this, we navigate
to the root of the workspace, and execute `catkin_make`.

```bash
cd ~/catkin_ws/
catkin_make
```

If you receive an error saying `catkin_make: command not found` then your
environment is not setup correctly. Did you `source` the correct setup.bash file
(the one in your workspace assuming your workspace was created correctly)? If
you receive no errors, then everything worked!

If your workspace is not located at `~/catkin_ws/` then all of the above should
be adjusted accordingly.

## Running the demo ##

The demo can simply be run using

```bash
roslaunch trep_puppet_demo puppet_sim.launch vis:=true
```

setting the last part to `vis:=false` will prevent `rviz` from starting up. This
could be nice when exploring how the pieces of this demo fit together as fewer
nodes and connections will actually be created.

If you get errors like

```bash
roslaunch: command not found
```

or

```bash
[puppet_sim.launch] is neither a launch file in package [trep_puppet_demo] nor is [trep_puppet_demo] a launch file name
The traceback for the exception was written to the log file
```

then something is wrong with your environment setup. Check your environment
variables with `env |grep ROS`, and be sure that you have properly sourced the
`setup.bash` file in your workspace. If your variables are still incorrect, or
you are still getting the above errors, try carefully re-following the
[catkin workspace creation tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
You could also try deleting your workspace `devel/` directory with `rm -r
~/catkin_ws/devel` and re-building with `catkin_make`.


## Things to explore ##

Below is a list of things that would be worth exploring

1. Launch the demo, and use ROS console tools to investigate how the pieces fit
   together (just like in class)

2. Practice using some of the `rqt` tools
    * `rosrun rqt_graph rqt_graph` Shows nodes, and their topic connections. See
      what happens when you toggle things like "Hide Debug" or "Group
      namespaces" at the top. Note that the first search box at the top is for
      picking individual nodes you are interested in, and the second search box
      is for picking topics you are interested in. They both accept a
      comma-separated list of topic/node names or regular expressions. For
      example, we can put `/,-.*rviz,-.*rqt,-.*rosout` in the first box,
      `/joint_states,/tf, /visualization_markers, /puppet_controls/.*` in the
      second box, and uncheck "Hide Debug" to get a nice looking block diagram
      of how the pieces actually fit together.
<p>
<br>
<img src="./images/rosgraph-resize.png" alt="Useful ROS graph" width="640"
      style="margin-left:auto; margin-right:auto; display:block;"/>
<br>
</p>

    * `rosrun rqt_plot rqt_plot /joint_states/position[1]` can be used to plot
      one of the left shoulder angles.

    * `rosrun rqt_tf_tree rqt_tf_tree` can show you how all of the coordinate
      systems in the ROS world fit together. `rosrun tf view_frames` can be used
      to generate a PDF of the `/tf` tree.

    * `rosrun rqt_console rqt_console` provides a quick console for viewing,
      searching, and controlling ROS logging messages. Try opening this, and
      then clicking the gear in the upper right. Then set the `rosout` logger
      for `/marker_controls` node to the *Debug* level. Notice that green
      printouts that are preceded by "[DEBUG]" now show up both in `rqt_console`
      and in the terminal that launched the demo.

3. Play around with toggling the different displays in `rviz` to get a feel for
   what they are. Of special interest are the settings for the *TF* display and
   the *RobotModel* display.

4. Take a glance at the [launch/puppet_sim.launch](./launch/puppet_sim.launch)
   file. Look at all of the *tags* and *attributes* that are set in the file,
   then check out the
   [roslaunch XML specifications](http://wiki.ros.org/roslaunch/XML) to try and
   understand what is going in this file. Specifically, look at the
   [tag reference](http://wiki.ros.org/roslaunch/XML) to see what the tags and
   their corresponding attributes mean.

5. After looking at the launch file, try launching the demo with different
   settings for the [<arg> tags](http://wiki.ros.org/roslaunch/XML/arg) (and
   thus, the corresponding 
   [substitution args](http://wiki.ros.org/roslaunch/XML#substitution_args)).

6. Take a look at the [urdf/manual_puppet.urdf](./urdf/manual_puppet.urdf) file,
   and the [wiki description URDFs](http://wiki.ros.org/urdf) and the
   corresponding [XML specification](http://wiki.ros.org/urdf/XML) for a
   URDF. Can you understand how this relates to the *TF* tree? How about how
   this fits together with `/joint_states` message? One helpful tool for
   visualizing a URDF is the `urdf_to_graphiz` command line tool. In Ubuntu
   14.04 and ROS Indigo, this tool is in the
   [liburdfdom-tools](https://launchpad.net/ubuntu/trusty/+package/liburdfdom-tools)
   deb package, and it is no longer part of the `urdfdom` ROS
   package. Therefore, it must be installed with
   ```bash
   sudo apt-get install liburdfdom-tools
   ```
   Then running
   ```bash
   urdf_to_graphiz ~/catkin_ws/src/trep_puppet_demo/urdf/manual_puppet.urdf
   ```
   will create a PDF visualization of the URDF in your current working directory.






[trep]: http://murpheylab.github.io/trep/
[ejo]: http://nxr.northwestern.edu/people/elliot-johnson
[pypi]: https://pypi.python.org/pypi/
[pip]:https://pypi.python.org/pypi/pip
[easy]: http://pythonhosted.org//setuptools/easy_install.html
[tinstall]: http://murpheylab.github.io/trep/install/
