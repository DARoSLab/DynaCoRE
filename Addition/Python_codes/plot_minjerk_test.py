import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 2


def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index


    file_path = os.getcwd() + "/../../experiment_data_check/"
    
    ## read files
    data_minj_pos = \
    np.genfromtxt(file_path+'minj_pos.txt', delimiter=None, dtype=(float))
    data_minj_vel = \
    np.genfromtxt(file_path+'minj_vel.txt', delimiter=None, dtype=(float))
    data_minj_acc = \
    np.genfromtxt(file_path+'minj_acc.txt', delimiter=None, dtype=(float))    
    
   
    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))
    
    st_idx = 1
    end_idx = len(data_x) - 10
    data_x = data_x[st_idx:end_idx]

    axes = plt.gca()
    
    ## plot pos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('min jerk positions')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_minj_pos[st_idx:end_idx,i-1], "b-")

        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1    

   
    ## plot vel
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('min jerk velocities')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_minj_vel[st_idx:end_idx,i-1], "b-")

        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1    


    ## plot acc
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('min jerk acc')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_minj_acc[st_idx:end_idx,i-1], "b-")

        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1            


if __name__ == "__main__":
    create_figures()
    plt.show()

