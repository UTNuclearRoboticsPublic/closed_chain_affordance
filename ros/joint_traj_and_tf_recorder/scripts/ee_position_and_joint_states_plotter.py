import pandas as pd
import os
import matplotlib.pyplot as plt
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
df_pred = pd.read_csv(relative_filepath)
df_act = pd.read_csv(relative_filepath2)
df_act['Timestamp'] = df_act['Timestamp'] - df_act['Timestamp'].iloc[0] # adjust timestamps such that we start from 0

# Plot 3D scatter plot
fig = plt.figure()

# Plot for EE position
ax_ee = fig.add_subplot(projection="3d")
scatter_pred = ax_ee.scatter(df_pred['Pred EE x']*1000, df_pred['Pred EE y']*1000, df_pred['Pred EE z']*1000, c='blue', marker='o', label='Predicted')
scatter_act = ax_ee.scatter(df_act['Act EE x']*1000, df_act['Act EE y']*1000, df_act['Act EE z']*1000, c='black', marker='o', label='Actual')
ax_ee.plot(df_pred['Pred EE x']*1000, df_pred['Pred EE y']*1000, df_pred['Pred EE z']*1000, c='blue', linestyle='-', linewidth=2, label='_nolegend_')
ax_ee.plot(df_act['Act EE x']*1000, df_act['Act EE y']*1000, df_act['Act EE z']*1000, c='black', linestyle='-', linewidth=2, label='_nolegend_')
ax_ee.set_xlabel('x (mm)')
ax_ee.set_ylabel('y (mm)')
ax_ee.set_zlabel('z (mm)')
ax_ee.legend()
ax_ee.set_title('EE Position - Turning a Valve')
set_axes_equal(ax_ee)
ax_ee.set_aspect('equal')

# Plot for joint states
fig2 = plt.figure(2)
ax_joint_states = fig2.add_subplot()
# for joint in df_pred.columns[:6]:  # Assuming first 6 columns are joint states
#     ax_joint_states.plot(df_pred['Timestamp'], df_pred[joint], label=joint)

num_samples = len(df_act)
manual_timestamps = np.arange(0, num_samples * 0.08, 0.08)
df_act['Timestamp'] = manual_timestamps

for joint in df_act.columns[:6]:  # Assuming first 6 columns are joint states
    ax_joint_states.plot(df_act['Timestamp'], df_act[joint], label=joint)
    # ax_joint_states.plot(df_act['Timestamp'], df_act[joint], label=f'act_{joint}')
# for joint in df_pred.columns[:6]:  # Assuming first 6 columns are joint states
#     ax_joint_states.plot(df_pred['Timestamp'], df_pred[joint], label=f'pred_{joint}')
ax_joint_states.set_xlabel('time (s)')
ax_joint_states.set_ylabel('joint states (rad)')
ax_joint_states.legend()
ax_joint_states.set_title('Actual Joint States vs. Time - Turning a Valve')

# Adjust layout
plt.tight_layout()

# Show the plot
plt.show()
