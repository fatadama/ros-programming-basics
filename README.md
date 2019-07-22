# ROS programming basics

* [Debugging with QTcreator](#debug-with-qtcreator)

# Debug with QTCreator

Build a relevant package using the compile option `-DCMAKE_BUILD_TYPE=Debug`:

```
> catkin build -DCMAKE_BUILD_TYPE=Debug <package name>
```

You can also change your cmake file to build the debugging configuration, but why would you want to do that?

Open QTCreator, open your file, and place breakpoints or whatever you want.
Launch the package using roslaunch or whatever.
I don't know if you can use this with a nodelet.
In QTCreator, go to Debug->Start Debugging->Attach to running process.
Select the running process.
You should now be debugging the process.

If you get a message "ptrace operation not allowed", you need to run this command:

```
> echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope
```

You can permanently chance the relevant setting at your own risk; see the `ptrace error fix` link.

Relevant links:
* [Debugging a ROS package with QTCreator](https://answers.ros.org/question/34966/debugging-ros-package-with-qtcreator/)
* [ptrace error fix](https://askubuntu.com/questions/41629/after-upgrade-gdb-wont-attach-to-process)
* [How to debug executable built with catkin_make without roslaunch](https://answers.ros.org/question/200155/how-to-debug-executable-built-with-catkin_make-without-roslaunch/)

# Profiling

## Valgrind

Profiling with `valgrind` is relatively straightforward.
In a standard ROS1 launch file, use the `launch-prefix` option to run your node with `valgrind`.

Note that `valgrind` does not seem to be compatible with all C++ standard libraries (or may not be depending on your system).

## Intel vTune Amplifier

* Install the Intel vTune Amplifier as root. The default install directory is `/opt/intel/vtune_amplifier`.
* Source the file `/opt/intel/vtune_amplifier/amplxe-vars.sh`
* Launch the GUI by calling `amplxe-gui` from a command line in the sourced workspace
* Create a new project
* The easiest way to run the profiler is to attach it to a running system. Under the "What" tab, click on "Launch application" and change it to "Attach to process". The "Process name" should just be the name of your node if you're using a node. I don't know about nodelets.
* When your node is running, you can attach the profiler and click the big Run button.
* The "Top-down tree" view seems to be a useful starting point for analyzing results.
