import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R

def extract_quaternions(csv_file, x_col, y_col, z_col, w_col):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(csv_file)

    # Concatenate the x, y, z, and w columns into an array
    quaternions_array = df[[x_col, y_col, z_col, w_col]].values

    # Convert the array to quaternions
    euler = [R.from_quat(row).as_euler('yzx') for row in quaternions_array]
    df = pd.DataFrame(euler, columns=["Euler Y", "Euler Z", "Euler X"])
    df.to_csv("euler.csv", index=False)


    return quaternions_array

# Example usage
csv_file = "your_file.csv"
x_column = "Pred EE x_or"
y_column = "Pred EE y_or"
z_column = "Pred EE z_or"
w_column = "Pred EE_w_or"

quaternions = extract_quaternions("pred_tf_and_joint_states_data.csv", x_column, y_column, z_column, w_column)
print(quaternions)
