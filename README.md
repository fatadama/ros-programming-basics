# ROS programming basics

* [Command line build options](#useful-command-line-build-options)
* [Debugging with gdb](#debug-with-gdb-directly)
* [Debugging with QTcreator](#debug-with-qtcreator)
* [Profiling](#profiling)

# Useful command line build options

* To build a package without rebuilding its dependencies, use `catkin build --no-deps <package>`
* To build a package using Debug settings, use `catkin build -DCMAKE_BUILD_TYPE=Debug <package>`

# Debug with gdb directly
In a ROS launch file, in your node's XML, add a launch prefix:
```xml
<node name="test_name" pkg="pkg_name"
    type = "node_name"
    output="screen"
    launch-prefix="xterm -e gdb -ex run --args"
    respawn="true">
```

This will launch the node in a separate xterm window with the debugger.
If the application closes unexpectedly, the `gdb` interface will be brought up.
Here are basics for using `gdb`:

* You can run `bt` to perform a backtrace of the crash. This will list the functions that crashed and its parents.
* To enter a particular function, use `frame [n]` where `[n]` is the number associated with your function in the `bt` result
* To evaluate an expression, use `print [exp]` or `p [exp]` to get the value of variables in the frame you are in
* To get the list of local variables in a frame, use `info locals`
* To get the arguments in a frame, use `info args`

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

* Install the Intel vTune Amplifier as root. The default install directory is `/opt/intel/vtune_amplifier` or `/home/$USER/intel/vtune_amplifier` if installing as non-root.
* Source the file `/opt/intel/vtune_amplifier/amplxe-vars.sh`
* Launch the GUI by calling `amplxe-gui` from a command line in the sourced workspace
* Create a new project
* The easiest way to run the profiler is to attach it to a running system. Under the "What" tab, click on "Launch application" and change it to "Attach to process". The "Process name" should just be the name of your node if you're using a node. I don't know about nodelets.
* When your node is running, you can attach the profiler and click the big Run button.
* The "Top-down tree" view seems to be a useful starting point for analyzing results.

Link: [Intel getting started on Linux guide](https://software.intel.com/en-us/get-started-with-vtune-linux-os)
