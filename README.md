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
