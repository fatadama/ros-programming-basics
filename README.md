# ROS programming basics

* [Command line build options](#useful-command-line-build-options)
* [Debugging with gdb](#debug-with-gdb-directly)
* [Debugging with QTcreator](#debug-with-qtcreator)
* [Profiling](#profiling)
* [Git](#git)
* [Code coverage](#code-coverage)

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

Build a relevant package using the compile option `--cmake-args -DCMAKE_BUILD_TYPE=Debug`:

```
> catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug -- <package name>
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

# Git

`git` is important.
* To undo a specific commit only, use `git cherry-pick`

# Code coverage

In software development you may need to evaluate code coverage.
This is commonly done using the `gcov` tool, but I have not found its use with ROS to be well-documented.
Here is a very short description of how to perform code coverage analysis of a ROS package.

## Things you will need:
* A ROS package you want to analyze
* (Probably) some code that calls it, like a Gtest or unittest

## Directions you will follow:
* Configure your catkin workspace. From the workspace root, run `catkin config --cmake-args -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_LD_FLAGS="--coverage"`, which will configure your package to generate extra stuff for gcov when it runs.
* Build your package using `catkin build <package>`
* Run your package tests (if not tests, then run the exectutables manually) using, e.g., `catkin test_results <package>`
* Find the `*.gcda` files generated by gcov, using, e.g., `find ./ -iname "*gcda" -type f` from your workspace root
* Using `lcov` to generate a readable performance report
    * `lcov --directory path/to/gcda/files/ --capture --output-file app.info` where `path/to/gcda/files/` contains the gcda files for your package. This command generates a file app.info in your working directory
    * `genhtml app.info` generates an html file you can read
    * This process creates a table showing the fraction of lines and the fraction of functions your files called in all the libraries it used. Look for the libraries you're interested in and read the results.
* **To get branch coverage results** - branch coverage is disabled by default in `lcov`. Run with the following options:
    * `lcov --directory path/to/gcda/files/ --capture --output-file app.info --rc lcov_branch_coverage=1`
    * `genhtml app.info --rc genhtml_branch_coverage=1`

Here is a very simple shell script that runs in folder `coverage` in your workspace. Run ./coverage.sh <pkg_name> followed by 1 to do a clean build. This script cleans the build folder, runs any tests in your CMakeLists.txt, and processes the results. It suppresses output for libraries not in your workspace.
```bash
#!/bin/bash

# package name
PKG_NAME=$1
# rebuild
REBUILD=${2:-0}

cd "../"
# build and run tests
remove existing
if [ ! $REBUILD == 0 ]
then 
  rm -rf build
  catkin build $PKG_NAME
  catkin run_tests $PKG_NAME --no-deps
fi
cd "coverage"
# if directory exists, continue
if [ -d "../build/$PKG_NAME/CMakeFiles" ]
then
  # run lcov, ignoring non-source library calls
  lcov --base-directory ../src/ --directory "../build/$PKG_NAME/CMakeFiles/" --capture --output-file "$PKG_NAME.info" -rc lcov_branch_coverage=1 --no-external
  # gen html with branch coverage
  genhtml "$PKG_NAME.info" --rc genhtml_branch_coverage=1 -o $PKG_NAME
fi
```
