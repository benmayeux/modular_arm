import dearpygui.dearpygui as dpg
# https://dearpygui.readthedocs.io/en/latest/about/what-why.html
from math import sin, cos

dpg.create_context()  # important line needed at the beginning of every dpg script

"""External file stuff"""
filename = 'text'
modules = 1  # keeps track of previous amount of modules for button deletion

"""Helper Functions"""


def menubar():  # makes the menu bar, such that it's consistent across windows
    with dpg.menu_bar():
        with dpg.menu(label="Menu"):
            dpg.add_menu_item(label="Exit", callback=lambda: dpg.destroy_context())
        with dpg.menu(label="Settings"):
            dpg.add_menu_item(label="Toggle Fullscreen", callback=lambda: dpg.toggle_viewport_fullscreen())


def save_callback(sender, app_data, user_data):  # example function for obtaining data from widget
    print(dpg.get_value(user_data))


def window_change(sender, app_data, user_data):
    # print(f"sender is: {sender}")
    # print(f"app_data is: {user_data}")
    dpg.configure_item(user_data[0], show=False)
    dpg.configure_item(user_data[1], show=True)
    dpg.set_primary_window(user_data[1], True)


# some fake data rn to test update plot
plot_t = [0]
plot_datay = [0]
frames = 0


def update_plot(frame_count):  # not generalized yet
    # updating plot data
    if len(plot_t) > 200:
        plot_t.pop(0)
        plot_datay.pop(0)
    plot_t.append(plot_t[-1] + 0.5)
    plot_datay.append(cos(3 * 3.14 * plot_t[-1] / 180))

    # plotting new data
    dpg.set_value('series_tag', [plot_t, plot_datay])
    dpg.fit_axis_data('x_axis')
    # dpg.fit_axis_data('y_axis')
    dpg.set_item_label('series_tag', "t=" + str(frame_count))  # not necessary


def update_custom():  # updates the custom window buttons based on the number of modules
    global modules
    file = open(filename, 'r')
    content = file.read()
    file.close()
    for i in range(modules):
        dpg.delete_item("cus_group" + str(i))
    for i in range(int(content)):
        with dpg.group(horizontal=True, width=110, parent="cus_window", tag="cus_group" + str(i)):
            dpg.add_input_float(label="Joint " + str(i + 1), tag="cus_joint" + str(i), default_value=0, step=0.01)
            dpg.add_button(label="Set", callback=save_callback, user_data="cus_joint" + str(i), tag="cus_set" + str(i))
    modules = int(content)


"""GUI structure"""

# load images
RRR_width, RRR_height, RRR_channels, RRR_data = dpg.load_image("RRR.png")
cus_width, cus_height, cus_channels, cus_data = dpg.load_image("custom.png")
with dpg.texture_registry(show=False):
    dpg.add_static_texture(width=RRR_width, height=RRR_height, default_value=RRR_data, tag="RRR_image")
    dpg.add_static_texture(width=cus_width, height=cus_height, default_value=cus_data, tag="cus_image")

with dpg.window(tag="Primary Window"):
    menubar()
    dpg.add_text("Select your arm config")
    dpg.add_separator()
    with dpg.group(horizontal=True):
        with dpg.group():
            dpg.add_image("RRR_image")
            dpg.add_button(label="Select###RRR", callback=window_change, user_data=["Primary Window", "RRR_window"])

        with dpg.group():
            dpg.add_image("cus_image")
            dpg.add_button(label="Select###cus", callback=window_change, user_data=["Primary Window", "cus_window"])

with dpg.window(label="RRR", modal=False, show=False, tag="RRR_window", no_title_bar=False):
    menubar()
    dpg.add_button(label="Back", callback=window_change, user_data=["RRR_window", "Primary Window"])
    dpg.add_separator()

    with dpg.group(width=110):  # joint controls
        with dpg.group(horizontal=True):
            dpg.add_input_float(tag="joint1RRR", label="Joint 1", default_value=0, step=0.01)
            dpg.add_button(label="Set", callback=save_callback, user_data="joint1RRR")
        with dpg.group(horizontal=True):
            dpg.add_input_float(tag="joint2RRR", label="Joint 2", default_value=0, step=0.01)
            dpg.add_button(label="Set", callback=save_callback, user_data="joint2RRR")
        with dpg.group(horizontal=True):
            dpg.add_input_float(tag="joint3RRR", label="Joint 3", default_value=0, step=0.01)
            dpg.add_button(label="Set", callback=save_callback, user_data="joint3RRR")

    with dpg.collapsing_header(label="Data Collection"):  # graphs
        with dpg.tree_node(label="Torques"):
            # create plot
            with dpg.plot(label="Line Series", height=400, width=400):
                # optionally create legend
                dpg.add_plot_legend()

                # REQUIRED: create x and y axes
                dpg.add_plot_axis(dpg.mvXAxis, label="x", tag='x_axis')
                dpg.add_plot_axis(dpg.mvYAxis, label="y", tag="y_axis")

                # series belong to a y axis
                dpg.add_line_series(plot_t, plot_datay, label="t=0", parent="y_axis", tag="series_tag")

with dpg.window(label="Custom", show=False, tag="cus_window"):
    menubar()
    with dpg.group():
        dpg.add_button(label="Back", callback=window_change, user_data=["cus_window", "Primary Window"])
        dpg.add_button(label="Scan Parts", callback=update_custom)
    dpg.add_separator()

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

    update_plot(frames)  # custom function to update a plot
    frames += 1  # keeping track of frames
    dpg.render_dearpygui_frame()  # render the frame

dpg.destroy_context()  # kill everything on exit
