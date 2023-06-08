# 2023-RobotTechnology-LineMarkingRobot
This is the final project for Robotic Technology course of Tongji University in 2023.

# How to run controller in your IDE 

## PyCharm

[PyCharm](https://www.jetbrains.com/pycharm) is a cross-platform integrated development environment (IDE), specifically for the Python language. It provides code analysis, a graphical debugger, an integrated unit tester and integration with version control systems (VCSes).

[PyCharm](https://www.jetbrains.com/pycharm) is a possible alternative to using Webots built-in editor for Python. This chapter explains step-by-step how to configure [PyCharm](https://www.jetbrains.com/pycharm) to edit a Python controller and run it. Although this chapter focuses on [PyCharm](https://www.jetbrains.com/pycharm), you should be able to configure any other Python IDE in a similar way.

### Creation of the PyCharm Project

Once PyCharm is started, click on `Open` and then select the directory of the Webots robot controller that you want to modify. As an example, the `driver` sample controller is used here.

![PyCharm Open File](https://raw.githubusercontent.com/cyberbotics/webots/released/docs/guide/images/pycharm_open.thumbnail.jpg)

### Open controller in PyCharm

In order to use the Webots Python API, it should be added to the project. This can be done from the `File` / `Settings` menu. In the `Settings` window, select the `Project` / `Project Structure` tab, then, the `Add Content Root` button can be used to add a new folder to the path, select the `WEBOTS_HOME/lib/controller/python38` folder (or any other Python version).

![PyCharm Add Library](https://raw.githubusercontent.com/cyberbotics/webots/released/docs/guide/images/pycharm_add_lib.thumbnail.jpg)

​                                              _Addition of the Webots controller library_

The Webots Python API depends on the Webots CPP API, therefore, the path need to be modifed to include the Webots `lib` directory. This can be done from the `Run` / `Edit Configurations` menu. In the `Run Configurations` windows, press the `+` button and then select `Python`, then set the `Script path` to point to your python file and in the `Environment variables` define the path variable (adjust it if needed, with your actual Webots installation folder):

Windows

```bash
Path=C:\Program Files\Webots\lib\controller\;C:\Program Files\Webots\msys64\mingw64\bin\;C:\Program Files\Webots\msys64\mingw64\bin\cpp
```

If you are using other libraries (e.g., the `vehicle libraries`, `DARwIn-OP library`, etc.), the path to the corresponding shared libraries should be added as well.

![PyCharm Add Path](https://raw.githubusercontent.com/cyberbotics/webots/released/docs/guide/images/pycharm_path.thumbnail.jpg)

​                                          _Addition of the Webots libraries to the path_

### Run the Controller

Once the [PyCharm](https://www.jetbrains.com/pycharm) project configured, you can start Webots and open the desired world. To allow [PyCharm](https://www.jetbrains.com/pycharm) to start the controller instead of Webots, set the controller of the robot to `<extern>` and set the environment variables as explained in the [Running Extern Robot Controllers](https://cyberbotics.com/doc/guide/running-extern-robot-controllers#environment-variables) chapter.

![PyCharm Webots](https://raw.githubusercontent.com/cyberbotics/webots/released/docs/guide/images/pycharm_webots.thumbnail.jpg)

​                                                           _Robot controller to external_

The controller can now be started from [PyCharm](https://www.jetbrains.com/pycharm) from the `Run` menu (if not already done, start the simulation in Webots).

![PyCharm Run](https://raw.githubusercontent.com/cyberbotics/webots/released/docs/guide/images/pycharm_run.thumbnail.jpg)

​                                                        _Run controller from PyCharm_



   
