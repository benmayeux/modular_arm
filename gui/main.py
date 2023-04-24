import math

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
modelType = None  # keeps track of what robot model should be used

matplotInteract = False  # for rendering the interactive matplot
remakePyPlot = False  # to remake the plot inside te main thread (callbacks are separate threads)

dpg.create_context()  # important line needed at the beginning of every dpg script

commsOpen = False
try:
    cs = commandSender(4)  # initialize serial command sender by COM port
    commsOpen = True
    print("Serial connected!")
except:
    print("No comms")

"""Testing stuff"""
filename = 'text'  # for testing updating custom window

# some fake data to test dynamic plots
plot_t = [0]
plot_datay = [0]


def update_fake_data():  # to create dynamic fake data, called in running loop
    if len(plot_t) > 200:
        plot_t.pop(0)
        plot_datay.pop(0)
    plot_t.append(plot_t[-1] + 0.5)
    plot_datay.append(cos(3 * 3.14 * plot_t[-1] / 180))

"""MATPLOT Simulation Stuff"""
fig_width = 4
fig_height = 4
plt.rcParams['figure.figsize'] = [fig_width, fig_height]
robot = rtb.models.DH.Panda()  # create a robot
robot.q = robot.qz  # set the robot configuration
pyplot = PyPlot() # create a PyPlot backend

def makeRobot(q):
    # print(modelType)
    global robot, pyplot, remakePyPlot

    if modelType == "RRR":
        robot = rtb.DHRobot([
            rtb.RevoluteDH(d=0.333, alpha=math.pi/2),
            rtb.RevoluteDH(a=0.333),
            rtb.RevoluteDH(a=0.333)
        ], name="RRR")
        robot.q = q

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

    pyplot.add(robot, show=False)     # add the robot to the backend, HAD TO CHANGE LIBRARY CODE TO INCLUDE SHOW OPTION
    matplot = convertFigToImage(fig)

    return matplot, fig, ax

matplot, fig, ax = makePyPlot()


"""Helper and Callback Functions"""
def windowMatPlot():  # to activate the interactive MatPlot
    global matplotInteract
    matplotInteract = True

def update_serial_data():
    torque_value = None

    try:

        lines = cs.arduino.read_all().decode('utf-8').strip().split('\n')

        for line in lines:
            if line.startswith("Received new command:"):
                name = str(line.split(":")[1].strip())
                print("Serial received the Command: " + name)
            elif line.startswith("Values of:"):
                values = line.split(":")[1].strip()
                print("Serial received the Values: " + values)
            elif line.startswith("Torque:"):
                torque_value = float(line.split(":")[1].strip())
            elif line.startswith("Num Modules:"):
                modules = float(line.split(":")[1].strip())
            # else:
            #     if line != "":
            #         print(line)
    except:
        #print("decode error")
        return

    if torque_value is not None:
        if len(plot_t) > 200:
            plot_t.pop(0)
            plot_datay.pop(0)
        plot_t.append(plot_t[-1] + 0.5)
        plot_datay.append(torque_value)

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


def addJointControlInput(config_name, n_modules):
    with dpg.collapsing_header(label="Joint Control", default_open=True):
        with dpg.group(width=110):
            for i in range(n_modules):
                with dpg.group(horizontal=True, tag="joint" + str(i) + config_name + "Group"):
                    dpg.add_input_float(label="Joint " + str(i), tag="joint" + str(i) + config_name,
                                        default_value=0, step=0.01)
                    dpg.add_button(label="Set", callback=sendSerialInput, user_data=["joint" + str(i) + config_name,"setJointPos",i])
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
        if len(y_data) == 1:
            dpg.add_line_series(x_data, y_data, label=y_name, parent=name + "_y_axis", tag=name)
        else:
            for i in range(len(y_data)):
                dpg.add_line_series(x_data, y_data[i], label=y_name + " " + str(i + 1), parent=name + "_y_axis",
                                    tag=name + str(i + 1))


def sendSerialInput(sender, app_data, user_data):  # function for obtaining data from widget and serial sending
    relatedTag = user_data[0]
    relatedValue = dpg.get_value(relatedTag)

    commandName = user_data[1]
    if commandName == "setTaskPos":
        # x=[0], y =[2], z=[1]
        x = commandName + "(" + str(relatedValue[0]) + "," + str(relatedValue[2]) + "," + str(relatedValue[1]) + ")"
    elif commandName == "setJointPos":
        if relatedValue > 50:
            warningBox("Warning: Joint Limit Exceeded", "You have exceeded the joint limit, try a new value")
            return
        x = commandName + "(" + str(user_data[2]) + "," + str(relatedValue) + ")"

        # update matplot for robot sim
        currentQ = robot.q
        #print("Old Q: " + str(currentQ))
        currentQ[user_data[2]] = relatedValue
        #print("New Q: " + str(currentQ))
        makeRobot(currentQ)
    else:
        x = ""

    if commsOpen:
        cs.write(x)
        print("Sent Serial:" + x)
    else:
        print(x)



def window_change(sender, app_data, user_data):  # callback for changing windows
    # print(f"sender is: {sender}")
    # print(f"app_data is: {user_data}")
    newWindow = user_data[1]
    dpg.configure_item(user_data[0], show=False)
    dpg.configure_item(newWindow, show=True)
    dpg.set_primary_window(user_data[1], True)

    global modelType
    if newWindow == "RRR_window":
        modelType = "RRR"
        print(modelType)
        makeRobot([0,0,0])



def update_custom():  # callback for updating custom window buttons based on the number of modules
    if commsOpen:
        new_num_modules = serialModules
    else:
        file = open(filename, 'r')
        content = file.read()
        file.close()
        new_num_modules = int(content)

    dpg.delete_item("starting_cus")  # remove the text that informed user what to do at start
    global prevNumModules
    for i in range(prevNumModules):
        dpg.delete_item("cus_group" + str(i))

    for i in range(int(new_num_modules)):
        with dpg.group(horizontal=True, width=110, parent="cus_joints", tag="cus_group" + str(i)):
            dpg.add_input_float(label="Joint " + str(i), tag="cus_joint" + str(i), default_value=0, step=0.01)
            dpg.add_button(label="Set", callback=sendSerialInput, user_data=["cus_joint" + str(i),"setJointPos",i], tag="cus_set" + str(i))

    prevNumModules = new_num_modules  # update modules variable


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
    #print(app_data)
    dpg.set_value(user_data[0], app_data[0])
    dpg.set_value(user_data[1], app_data[2])
    dpg.set_value(user_data[2], app_data[1])


def update_plot(name, x_data, y_data):
    # plotting new data
    dpg.set_value(name, [x_data, y_data])
    dpg.fit_axis_data(name + "_x_axis")


"""GUI structure"""

# load images
RRR_width, RRR_height, RRR_channels, RRR_data = dpg.load_image("RRR.png")
cus_width, cus_height, cus_channels, cus_data = dpg.load_image("custom.png")

with dpg.texture_registry(show=False):
    dpg.add_static_texture(width=RRR_width, height=RRR_height, default_value=RRR_data, tag="RRR_image")
    dpg.add_static_texture(width=cus_width, height=cus_height, default_value=cus_data, tag="cus_image")
    dpg.add_raw_texture(fig_width * 100, fig_height * 100, matplot, format=dpg.mvFormat_Float_rgba, tag="matplot")

# Windows
with dpg.window(tag="Primary Window"):
    addMenubar()
    dpg.add_text("Select your arm config")
    dpg.add_separator()
    with dpg.group(horizontal=True):
        with dpg.group():
            dpg.add_text("RRR")
            dpg.add_image("RRR_image")
            dpg.add_button(label="Select###RRR", callback=window_change, user_data=["Primary Window", "RRR_window"])

        with dpg.group():
            dpg.add_text("Custom")
            dpg.add_image("cus_image")
            dpg.add_button(label="Select###cus", callback=window_change, user_data=["Primary Window", "cus_window"])

with dpg.window(label="RRR", modal=False, show=False, tag="RRR_window", no_title_bar=False):
    addMenubar()
    dpg.add_button(label="Back", callback=window_change, user_data=["RRR_window", "Primary Window"])
    dpg.add_separator()

    addJointControlInput("RRR", 3)
    addTaskSpaceInput("RRR")

    with dpg.collapsing_header(label="Robot Simulation"):
        dpg.add_image("matplot")
        with dpg.group(horizontal=True):
            # dpg.add_button(label="test", callback=updateMatPlot)
            dpg.add_button(label="interact", callback=windowMatPlot)

    with dpg.collapsing_header(label="Data Collection"):  # graphs
        """TODO: change data to be from serial inputs"""
        with dpg.tree_node(label="Torques"):
            addPlot("RRRTorque", "Time", "Torque", plot_t, plot_datay)
        with dpg.tree_node(label="Velocities"):
            addPlot("RRRVelocity", "Time", "Velocity", plot_t, plot_datay)

with dpg.window(label="Custom", show=False, tag="cus_window"):
    addMenubar()
    with dpg.group():
        dpg.add_button(label="Back", callback=window_change, user_data=["cus_window", "Primary Window"])
        dpg.add_button(label="Scan Parts", callback=update_custom)  # where the joint number update happens
    dpg.add_separator()

    with dpg.collapsing_header(label="Joint Control", default_open=True, tag="cus_joints"):
        dpg.add_text("Press Scan Parts", tag="starting_cus")

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
                update_serial_data()  # for serial reading testing
        else:
            update_fake_data()  # for dynamic graph testing
        update_plot("RRRTorque", plot_t, plot_datay)  # custom function to update a plot
        update_plot("RRRVelocity", plot_t, plot_datay)  # custom function to update a plot
        frames += 1  # keeping track of frames
        dpg.render_dearpygui_frame()  # render the frame

if commsOpen:
    cs.close()  # close serial com
dpg.destroy_context()  # kill everything on exit
