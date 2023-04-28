import math
import time

import dearpygui.dearpygui as dpg
import matplotlib
# https://dearpygui.readthedocs.io/en/latest/about/what-why.html
import numpy as np
from math import sin, cos
from matplotlib import pyplot as plt
from matplotlib.backend_bases import FigureCanvasBase
from matplotlib.backends.backend_agg import FigureCanvasAgg
from commandSender import commandSender
import roboticstoolbox as rtb
from RTBPyPlot import PyPlot  # copy of base library (rtb) but with changes

frames = 0  # keeps track of the number of frames
prevNumModules = 1  # keeps track of previous amount of modules for button deletion
serialModules = 0  # keeps track of serial reading for modules number
newNumModules = 0  # keeps track of current number of modules
modelType = None  # keeps track of what robot model should be used
start_time = round(time.time()*1000)  # for keeping track of time in plots

# some states
matplotInteract = False  # for rendering the interactive matplot
remakePyPlot = False  # to remake the plot inside te main thread (callbacks are separate threads)
plotMade = False  # to prevent update from happening before plots are created
pausedReadings = False  # to pause serial readings from updating plots

# All the supported models, key = model name, first value = number of modules, second value = robot model
# NOTE: if you want to add a menu image for that model, go add it in dpg.texture_registry
supportedModels = {"RRR": [3, rtb.DHRobot([
            rtb.RevoluteDH(d=0.333, alpha=math.pi / 2),
            rtb.RevoluteDH(a=0.333),
            rtb.RevoluteDH(a=0.333)
        ], name="RRR")],
                    "6DOF": [6, rtb.DHRobot([
            rtb.RevoluteDH(d=0.333, alpha=math.pi / 2),
            rtb.RevoluteDH(a=0.333),
            rtb.RevoluteDH(a=0.333),
            rtb.RevoluteDH(a=1),
            rtb.RevoluteDH(a=1),
            rtb.RevoluteDH(a=1)
        ], name="6DOF")]}

dpg.create_context()  # important line needed at the beginning of every dpg script

commsOpen = False
try:
    cs = commandSender(4)  # initialize serial command sender by COM port
    commsOpen = True
    print("Serial connected!")
except:
    print("No comms")

plot_t = []  # stores x-axis data (usually time)
plot_datay = []  # stores y-axis data (the interesting data like torque)
dataCategories = 2  # stores number of data categories per joint (torque and velocity = 2)
numOfDataSets = 3 * dataCategories  # stores number of data sets/expected graphs

# NOTE: make sure that the below are in numerical order starting from 0
torque_location = 0  # index for torque data in array
velocity_location = 1  # index for velocity data in array



def initializeDatasets(numOfSets):
    global plot_t, plot_datay
    plot_t.clear()
    plot_datay.clear()
    for i in range(numOfSets):
        plot_t.append([0])
        plot_datay.append([0])


initializeDatasets(20)  # big enough to hold all supported windows when they are initialized before running

"""Testing stuff (Called when comms are closed)"""
filename = 'text'  # for testing updating custom window


def update_fake_data():  # to create dynamic fake data, called in running loop
    for i in range(numOfDataSets):
        currentT = plot_t[i]
        currentY = plot_datay[i]
        if len(currentT) > 400:
            currentT.pop(0)
            currentY.pop(0)
        currentT.append(currentT[-1] + 0.5)
        # currentY.append(sin(3 * 3.14 * currentT[-1] / 180))
        currentY.append(np.random.random())


"""MATPLOT Simulation Stuff"""
fig_width = 4
fig_height = 4
plt.rcParams['figure.figsize'] = [fig_width, fig_height]
robot = rtb.models.DH.Panda()  # create a robot
robot.q = robot.qz  # set the robot configuration
pyplot = PyPlot()  # create a PyPlot backend


def windowMatPlot():  # to activate the interactive MatPlot
    global matplotInteract
    matplotInteract = True


def makeRobot(q):
    # print(modelType)
    global robot, pyplot, remakePyPlot

    for model in supportedModels:
        if model == modelType:
            try:
                robot = supportedModels[model][1]
                robot.q = q
            except:
                print("No model setup")

    remakePyPlot = True


def convertFigToImage(figure):
    canvas = FigureCanvasAgg(figure)
    canvas.draw()
    buf = canvas.buffer_rgba()
    plot_image = np.asarray(buf)
    return plot_image.astype(np.float32) / 255


def makePyPlot():
    pyplot.launch(show=False)  # setup pyplot fig and ax, HAD TO CHANGE LIBRARY CODE TO INCLUDE SHOW OPTION
    fig = pyplot.fig
    ax = pyplot.ax

    pyplot.add(robot, show=False)  # add the robot to the backend, HAD TO CHANGE LIBRARY CODE TO INCLUDE SHOW OPTION
    matplot = convertFigToImage(fig)

    return matplot, fig, ax


matplot, fig, ax = makePyPlot()  # initialize the plot

"""Other Helper and Callback Functions"""


def update_serial_data():
    # torque_value = None

    try:

        # lines = cs.arduino.read_all().decode('utf-8').strip().split('\n')
        #
        # for line in lines:
        #     if line.startswith("Received new command:"):
        #         name = str(line.split(":")[1].strip())
        #         print("Serial received the Command: " + name)
        #     elif line.startswith("Values of:"):
        #         values = line.split(":")[1].strip()
        #         print("Serial received the Values: " + values)
        #     elif line.startswith("Torque:"):
        #         torque_value = float(line.split(":")[1].strip())
        #     elif line.startswith("Num Modules:"):
        #         global serialModules
        #         serialModules = float(line.split(":")[1].strip())

        line = cs.arduino.read_until(expected=bytes("\n", 'utf-8')).decode('utf-8')

        numOfLines = int(line.split(",")[0].strip())
        global serialModules
        serialModules = numOfLines
        for i in range(numOfLines):
            line = cs.arduino.read_until(expected=bytes("\n", 'utf-8')).decode('utf-8').split(",")

            for j in range(len(line)):
                if j == 2:  # should be torque data in serial
                    currentT = plot_t[dataCategories * i + torque_location]
                    currentY = plot_datay[dataCategories * i + torque_location]

                    if len(currentT) > 200:
                        currentT.pop(0)
                        currentY.pop(0)
                    currentT.append(round(time.time()*1000) - start_time)
                    currentY.append(float(line[j].strip))
                if j == 3:  # should be velocity data
                    currentT = plot_t[dataCategories * i + velocity_location]
                    currentY = plot_datay[dataCategories * i + velocity_location]

                    if len(currentT) > 200:
                        currentT.pop(0)
                        currentY.pop(0)
                    currentT.append(round(time.time() * 1000) - start_time)
                    currentY.append(float(line[j].strip))

            # else:
            #     if line != "":
            #         print(line)
    except:
        # print("decode error")
        return

    # if torque_value is not None:
    #     torqueT = plot_t[0]
    #     torqueY = plot_datay[0]
    #     if len(torqueT) > 200:
    #         torqueT.pop(0)
    #         torqueY.pop(0)
    #     torqueT.append(torqueT[-1] + 0.5)
    #     torqueY.append(torque_value)


def sendSerialInput(sender, app_data, user_data):  # function for obtaining data from widget and serial sending
    relatedTag = user_data[0]
    relatedValue = dpg.get_value(relatedTag)

    commandName = user_data[1]
    if commandName == "setTaskPos":
        # x=[0], y =[2], z=[1]
        x = commandName + "," + str(relatedValue[0]) + "," + str(relatedValue[2]) + "," + str(relatedValue[1]) + ";"
    elif commandName == "setJointPos":
        if relatedValue > 50:
            warningBox("Warning: Joint Limit Exceeded", "You have exceeded the joint limit, try a new value")
            return
        x = commandName + "," + str(user_data[2]) + "," + str(relatedValue/0.017453*100) + ";"

        if modelType != "cus":
            # update matplot for robot sim
            currentQ = robot.q
            # print("Old Q: " + str(currentQ))
            currentQ[user_data[2]] = relatedValue
            # print("New Q: " + str(currentQ))
            makeRobot(currentQ)

    else:
        x = ""

    if commsOpen:
        cs.write(x)

        print("Sent Serial:" + x)
    else:
        print(x)


def warningBox(title, message):
    # guarantee these commands happen in the same frame
    with dpg.mutex():
        viewport_width = dpg.get_viewport_client_width()
        viewport_height = dpg.get_viewport_client_height()

        with dpg.window(label=title, modal=True, no_close=True) as modal_id:
            dpg.add_text(message)
            dpg.add_button(label="Ok", width=75, user_data=(modal_id, True), callback=lambda: dpg.delete_item(modal_id))

    # guarantee these commands happen in another frame
    dpg.split_frame()
    width = dpg.get_item_width(modal_id)
    height = dpg.get_item_height(modal_id)
    dpg.set_item_pos(modal_id, [viewport_width // 2 - width // 2, viewport_height // 2 - height // 2])


def addMenubar():  # makes the menu bar, such that it's consistent across windows
    with dpg.menu_bar():
        with dpg.menu(label="Menu"):
            dpg.add_menu_item(label="Exit", callback=lambda: dpg.destroy_context())
        with dpg.menu(label="Settings"):
            dpg.add_menu_item(label="Toggle Fullscreen", callback=lambda: dpg.toggle_viewport_fullscreen())
            dpg.add_menu_item(label="Pause Plots", callback=pausePlots)


def addJointControlInput(config_name, n_modules):
    with dpg.collapsing_header(label="Joint Control", default_open=True):
        with dpg.group(width=110):
            for i in range(n_modules):
                with dpg.group(horizontal=True, tag="joint" + str(i) + config_name + "Group"):
                    dpg.add_input_float(label="Joint " + str(i), tag="joint" + str(i) + config_name,
                                        default_value=0, step=0.01)
                    dpg.add_button(label="Set", callback=sendSerialInput,
                                   user_data=["joint" + str(i) + config_name, "setJointPos", i])
                    # user data = [tag of corresponding input_float, Serial command name, joint number]


def addTaskSpaceInput(config_name):
    with dpg.collapsing_header(label="Task Space Control"):
        dpg.add_3d_slider(label="Workspace (mm)", tag="task" + config_name, scale=0.3, callback=update_slider_inputs,
                          user_data=[config_name + "x", config_name + "y", config_name + "z"])
        with dpg.group(horizontal=True):
            dpg.add_input_float(label="x", tag=config_name + "x", callback=update_3d_slider,
                                user_data=["task" + config_name, "x"], width=110)
            dpg.add_input_float(label="y", tag=config_name + "y", callback=update_3d_slider,
                                user_data=["task" + config_name, "y"], width=110)
            dpg.add_input_float(label="z", tag=config_name + "z", callback=update_3d_slider,
                                user_data=["task" + config_name, "z"], width=110)
        dpg.add_button(label="Set", callback=sendSerialInput, user_data=["task" + config_name, "setTaskPos"])


def addPlot(name, x_name, y_name, x_data, y_data):
    """Make sure to use updatePlot function in running loop if dynamic"""
    with dpg.plot(label=name, height=400, width=400):
        # optionally create legend
        dpg.add_plot_legend()

        # REQUIRED: create x and y axes
        dpg.add_plot_axis(dpg.mvXAxis, label=x_name, tag=name + '_x_axis')
        dpg.add_plot_axis(dpg.mvYAxis, label=y_name, tag=name + "_y_axis")

        # series belong to a y axis
        # if len(y_data) == 1:
        #     dpg.add_line_series(x_data, y_data, label=y_name, parent=name + "_y_axis", tag=name)
        # else:
        #     for i in range(len(y_data)):
        #         try:
        #             dpg.add_line_series(x_data, y_data[i], label=y_name + " " + str(i + 1), parent=name + "_y_axis",
        #                                 tag=name + str(i + 1))
        #         except:
        #             print(x_data)
        #             print(y_data[i])
        #             print(name)
        #             print(y_name)
        if len(y_data) != 0:
            dpg.add_line_series(x_data, y_data, label=y_name, parent=name + "_y_axis", tag=name)


def addPlotSection(configName):
    dpg.delete_item(configName+"_plots")

    with dpg.collapsing_header(label="Data Collection", tag=configName+"_plots", parent=configName+"_window"):
        dpg.add_text("Live Data from Robot")

    global newNumModules, plotMade
    numberOfPlotsPerRow = 3
    with dpg.tree_node(label="Torques", parent=configName + "_plots"):
        moduleNumber = 0  # to keep track of which module plot is being created
        for i in range(math.ceil(newNumModules/numberOfPlotsPerRow)):
            with dpg.group(horizontal=True):
                for j in range(numberOfPlotsPerRow):
                    if moduleNumber <= newNumModules-1:
                        addPlot(configName + " Torque Joint" + str(moduleNumber), "Time", "Torque", plot_t[dataCategories * moduleNumber + torque_location],
                                plot_datay[dataCategories * moduleNumber + torque_location])
                    moduleNumber += 1
    with dpg.tree_node(label="Velocities", parent=configName + "_plots"):
        moduleNumber = 0  # to keep track of which module plot is being created
        for i in range(math.ceil(newNumModules/numberOfPlotsPerRow)):
            with dpg.group(horizontal=True):
                for j in range(numberOfPlotsPerRow):
                    if moduleNumber <= newNumModules-1:
                        addPlot(configName + " Velocity Joint" + str(moduleNumber), "Time", "Velocity",
                                plot_t[dataCategories * moduleNumber + velocity_location],
                                plot_datay[dataCategories * moduleNumber + velocity_location])
                    moduleNumber += 1
        # with dpg.group(horizontal=True):
        #     for i in range(newNumModules):
        #         addPlot(configName + " Velocity Joint" + str(i), "Time", "Velocity", plot_t[dataCategories * i + velocity_location],
        #                 plot_datay[dataCategories * i + velocity_location])
    plotMade = True


def window_change(sender, app_data, user_data):  # callback for changing windows
    # print(f"sender is: {sender}")
    # print(f"app_data is: {user_data}")
    newWindow = user_data[1]
    dpg.configure_item(user_data[0], show=False)
    dpg.configure_item(newWindow, show=True)
    dpg.set_primary_window(user_data[1], True)

    global modelType, numOfDataSets, dataCategories, newNumModules, plotMade, supportedModels
    plotMade = False

    for model in supportedModels:
        if newWindow == model+"_window":
            initializeSupportedWindowVariables(model, supportedModels[model][0])
    # if newWindow == "RRR_window":
    #     initializeSupportedWindowVariables("RRR", 3)  # for supported configs

    if newWindow == "cus_window":
        modelType = "cus"
        initialize_custom()

    elif newWindow == "Primary Window":
        modelType = None


def initializeSupportedWindowVariables(modelName, numModules):
    global modelType, numOfDataSets, dataCategories, newNumModules
    modelType = modelName
    print(modelType)
    makeRobot(np.zeros(numModules))
    newNumModules = numModules
    numOfDataSets = newNumModules * dataCategories
    initializeDatasets(numOfDataSets)
    addPlotSection(modelType)

def initialize_custom():
    dpg.delete_item("cus_joints")
    dpg.delete_item("cus_plots")

    with dpg.collapsing_header(label="Joint Control", default_open=True, tag="cus_joints", parent="cus_window"):
        dpg.add_text("Press Scan Parts", tag="starting_cus")

    with dpg.collapsing_header(label="Data Collection", tag="cus_plots", parent="cus_window"):
        dpg.add_text("Live Data from Robot")


def update_custom():  # callback for updating custom window buttons based on the number of modules
    global newNumModules, dataCategories, numOfDataSets, serialModules
    if commsOpen:
        newNumModules = serialModules
    else:
        file = open(filename, 'r')
        content = file.read()
        file.close()
        newNumModules = int(content)

    print("New num of modules: " + str(newNumModules))

    dpg.delete_item("starting_cus")  # remove the text that informed user what to do at start
    global prevNumModules
    for i in range(prevNumModules):
        dpg.delete_item("cus_group" + str(i))

    for i in range(int(newNumModules)):  # add joint input buttons
        with dpg.group(horizontal=True, width=110, parent="cus_joints", tag="cus_group" + str(i)):
            dpg.add_input_float(label="Joint " + str(i), tag="cus_joint" + str(i), default_value=0, step=0.01)
            dpg.add_button(label="Set", callback=sendSerialInput, user_data=["cus_joint" + str(i), "setJointPos", i],
                           tag="cus_set" + str(i))

    numOfDataSets = newNumModules * dataCategories
    initializeDatasets(numOfDataSets)
    addPlotSection("cus")
    prevNumModules = newNumModules  # update modules variable


def update_3d_slider(sender, app_data, user_data):  # callback to update slider based on float_inputs
    # slider values = [x z y]
    currVal = dpg.get_value(user_data[0])
    newVal = app_data
    if user_data[1] == "x":
        currVal[0] = newVal
    elif user_data[1] == "y":
        currVal[2] = newVal
    else:
        currVal[1] = newVal
    # print(currVal)
    dpg.set_value(user_data[0], currVal)


def update_slider_inputs(sender, app_data, user_data):  # callback to update slider inputs based on 3Dslider
    # slider values = [x z y]
    # print(app_data)
    dpg.set_value(user_data[0], app_data[0])
    dpg.set_value(user_data[1], app_data[2])
    dpg.set_value(user_data[2], app_data[1])


def update_plot(name, x_data, y_data):
    # plotting new data
    dpg.set_value(name, [x_data, y_data])
    dpg.fit_axis_data(name + "_x_axis")


def createSupportedWindow(modelName, numModules):
    with dpg.window(label=modelName, modal=False, show=False, tag=modelName+"_window", no_title_bar=False):
        addMenubar()
        dpg.add_button(label="Back", callback=window_change, user_data=[modelName+"_window", "Primary Window"])
        dpg.add_separator()

        addJointControlInput(modelName, numModules)
        addTaskSpaceInput(modelName)

        with dpg.collapsing_header(label="Robot Simulation"):
            dpg.add_image("matplot")
            with dpg.group(horizontal=True):
                # dpg.add_button(label="test", callback=updateMatPlot)
                dpg.add_button(label="interact", callback=windowMatPlot)

        with dpg.collapsing_header(label="Data Collection", tag=modelName+"_plots"):  # graphs
            dpg.add_text("Live Data from Robot")

def pausePlots():
    global pausedReadings
    pausedReadings = not pausedReadings


"""GUI structure"""

# load images
RRR_width, RRR_height, RRR_channels, RRR_data = dpg.load_image("RRR.png")
cus_width, cus_height, cus_channels, cus_data = dpg.load_image("custom.png")

with dpg.texture_registry(show=False):
    # menu images
    dpg.add_static_texture(width=RRR_width, height=RRR_height, default_value=RRR_data, tag="RRR_image")
    dpg.add_static_texture(width=cus_width, height=cus_height, default_value=cus_data, tag="cus_image")

    # matplot simulation
    dpg.add_raw_texture(fig_width * 100, fig_height * 100, matplot, format=dpg.mvFormat_Float_rgba, tag="matplot")

# Windows
with dpg.window(tag="Primary Window"):
    addMenubar()
    dpg.add_text("Select your arm config")
    dpg.add_separator()

    with dpg.group(horizontal=True):
        for model in supportedModels:
            with dpg.group():
                dpg.add_text(model)
                try:
                    dpg.add_image(model+"_image")
                except:
                    print("No such image")
                dpg.add_button(label="Select###"+model, callback=window_change, user_data=["Primary Window", model+"_window"])

        with dpg.group():
            dpg.add_text("Custom")
            dpg.add_image("cus_image")
            dpg.add_button(label="Select###cus", callback=window_change, user_data=["Primary Window", "cus_window"])

for model in supportedModels:
    createSupportedWindow(model, supportedModels[model][0])

with dpg.window(label="Custom", show=False, tag="cus_window"):
    addMenubar()
    with dpg.group():
        dpg.add_button(label="Back", callback=window_change, user_data=["cus_window", "Primary Window"])
        dpg.add_button(label="Scan Parts", callback=update_custom)  # where the joint number update happens
    dpg.add_separator()

    initialize_custom()

"""Final stuff to display GUI"""
dpg.create_viewport(title='Modular Arm', width=600, height=300)  # window created by OS to show GUI windows
dpg.setup_dearpygui()
dpg.show_viewport()
# dpg.toggle_viewport_fullscreen() # make viewport start fullscreen
dpg.set_primary_window("Primary Window", True)  # True = set in/fill up the viewport
# dpg.show_debug()
# dpg.show_about()
# dpg.show_metrics()

# below replaces, start_dearpygui()
while dpg.is_dearpygui_running():  # this starts the runtime loop
    # insert here any code you would like to run in the render loop
    # you can manually stop by using stop_dearpygui()
    # print("this will run every frame")

    if matplotInteract:  # to stop rendering dearpygui for matplot lib

        plt.ion()
        plt.show()
        while not (len(plt.get_fignums()) == 0):
            plt.pause(0.01)
            FigureCanvasBase(fig).flush_events()

        plt.ioff()
        matplotInteract = False

        # remake plot because image gets messed otherwise
        matplot, fig, ax = makePyPlot()

    elif remakePyPlot:  # to remake the plot, needs to be in main thread
        plt.close(pyplot.fig)
        matplot, fig, ax = makePyPlot()
        dpg.set_value("matplot", matplot)
        remakePyPlot = False
    else:
        if commsOpen:
            if frames % 50 == 0:
                cs.write("poll;")
                update_serial_data()  # for serial reading testing
        else:
            update_fake_data()  # for dynamic graph testing

        if modelType is not None and plotMade:
            if not pausedReadings:
                for i in range(int(newNumModules)):  # for each joint
                    update_plot(modelType + " Torque Joint" + str(i), plot_t[dataCategories * i + torque_location],
                                plot_datay[dataCategories * i + torque_location])  # custom function to update a plot
                    update_plot(modelType + " Velocity Joint" + str(i), plot_t[dataCategories * i + velocity_location],
                                plot_datay[dataCategories * i + velocity_location])

        frames += 1  # keeping track of frames
        dpg.render_dearpygui_frame()  # render the frame

if commsOpen:
    cs.close()  # close serial com
dpg.destroy_context()  # kill everything on exit
