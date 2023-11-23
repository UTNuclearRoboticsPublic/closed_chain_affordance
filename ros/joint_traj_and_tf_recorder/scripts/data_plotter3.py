import pandas as pd
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# Read CSV files
script_dir = os.path.dirname(os.path.abspath(__file__))
relative_filepath = os.path.join(script_dir, '..', 'data', 'pred_tf_and_joint_states_data.csv')
relative_filepath2 = os.path.join(script_dir, '..', 'data', 'act_tf_and_joint_states_data.csv')
df_pred = pd.read_csv(relative_filepath, usecols=['Pred EE x', 'Pred EE y', 'Pred EE z'])
df_act = pd.read_csv(relative_filepath2, usecols=['Act EE x', 'Act EE y', 'Act EE z'])

df_pred = df_pred*1000
df_act = df_act*1000

# Plot 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(projection="3d")

# Scatter plot for predicted data
scatter_pred = ax.scatter(df_pred['Pred EE x'], df_pred['Pred EE y'], df_pred['Pred EE z'], c='blue', marker='o', label='Predicted')

# Scatter plot for actual data
scatter_act = ax.scatter(df_act['Act EE x'], df_act['Act EE y'], df_act['Act EE z'], c='black', marker='o', label='Actual')

# Convert DataFrame columns to numpy arrays
x_vals_pred = df_pred['Pred EE x'].to_numpy()
y_vals_pred = df_pred['Pred EE y'].to_numpy()
z_vals_pred = df_pred['Pred EE z'].to_numpy()
x_vals_act = df_act['Act EE x'].to_numpy()
y_vals_act = df_act['Act EE y'].to_numpy()
z_vals_act = df_act['Act EE z'].to_numpy()

# Connect the points with a line for predicted data
ax.plot(x_vals_pred, y_vals_pred, z_vals_pred, c='blue', linestyle='-', linewidth=2, label='_nolegend_')  # Label set to '_nolegend_'

# Connect the points with a line for actual data
ax.plot(x_vals_act, y_vals_act, z_vals_act, c='black', linestyle='-', linewidth=2, label='_nolegend_')  # Label set to '_nolegend_'


# Set labels
ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_zlabel('z (mm)')

# Add legend for scatter
ax.legend()

# Add title
ax.set_title('EE Position - Pure rotation with no grasp')

set_axes_equal(ax)

# Show the plot
plt.show()
