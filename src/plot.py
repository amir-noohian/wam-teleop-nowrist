import matplotlib.pyplot as plt
import os
import sys
import numpy as np

def read_config(file_path):
    kinematics_vars = []
    dynamics_vars = []

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith("Kinematics data:"):
                kinematics_vars = line.split(":")[1].strip().split(", ")
            elif line.startswith("Dynamics data:"):
                dynamics_vars = line.split(":")[1].strip().split(", ")

    return kinematics_vars, dynamics_vars

def read_data(file_path, variable_names, dof=4):
    data_dict = {name: [] for name in variable_names}

    with open(file_path, 'r') as file:
        for line in file:
            values = list(map(float, line.strip().split(",")))
            data_dict[variable_names[0]].append(values[0])  # time

            idx = 1
            for var in variable_names[1:]:
                if idx + dof <= len(values):
                    data_dict[var].append(values[idx:idx + dof])
                    idx += dof

    for key in data_dict:
        data_dict[key] = np.array(data_dict[key])
    return data_dict

def plot_all_vars(kinematics_data, dynamics_data, dof=4):
    total_plots = (len(kinematics_data) - 1 + len(dynamics_data) - 1) * dof
    cols = 4
    rows = (total_plots + cols - 1) // cols
    plt.figure(figsize=(16, 3 * rows))

    plot_idx = 1
    for var, data in kinematics_data.items():
        if var == 'time':
            continue
        for j in range(dof):
            plt.subplot(rows, cols, plot_idx)
            plt.plot(kinematics_data['time'], data[:, j])
            plt.title(f"{var} [Joint {j+1}]")
            plt.xlabel("Time (s)")
            plt.ylabel(var)
            plot_idx += 1

    for var, data in dynamics_data.items():
        if var == 'time':
            continue
        for j in range(dof):
            plt.subplot(rows, cols, plot_idx)
            plt.plot(dynamics_data['time'], data[:, j])
            plt.title(f"{var} [Joint {j+1}]")
            plt.xlabel("Time (s)")
            plt.ylabel(var)
            plot_idx += 1

    plt.tight_layout()
    plt.show()

def calculate_nrmse(desired, feedback, normalization='range'):
    rmse = np.sqrt(np.mean((desired - feedback) ** 2))
    if normalization == 'range':
        range_desired = np.max(desired) - np.min(desired)
        nrmse = rmse / range_desired if range_desired != 0 else float('inf')
    elif normalization == 'mean':
        mean_desired = np.mean(desired)
        nrmse = rmse / mean_desired if mean_desired != 0 else float('inf')
    else:
        raise ValueError("Normalization method must be 'range' or 'mean'.")
    return nrmse

def calculate_rms(signal):
    return np.sqrt(np.mean(np.square(signal)))

def calculate_errors(kinematics_data, dynamics_data, dof=4):
    pos_nrmse_list = []
    pos_rms_list = []
    ext_rms_list = []
    impedance_list = []

    for joint in range(dof):
        pos_des = kinematics_data['desired joint pos'][:, joint]
        pos_feedback = kinematics_data['feedback joint pos'][:, joint]
        ext_torque = dynamics_data['external torque'][:, joint]

        pos_nrmse = calculate_nrmse(pos_des, pos_feedback)
        pos_rms = calculate_rms(pos_feedback)
        ext_rms = calculate_rms(ext_torque)
        impedance = ext_rms / pos_rms if pos_rms != 0 else float('inf')

        pos_nrmse_list.append(pos_nrmse)
        pos_rms_list.append(pos_rms)
        ext_rms_list.append(ext_rms)
        impedance_list.append(impedance)

    print("---- Impedance ----")
    for joint, value in enumerate(impedance_list):
        print(f"  Joint {joint+1}: {value:.4f}")

    print("\n---- nRMSE (Position) ----")
    for joint, value in enumerate(pos_nrmse_list):
        print(f"  Joint {joint+1}: {value:.4f}")

    print("\n---- RMS Position ----")
    for joint, value in enumerate(pos_rms_list):
        print(f"  Joint {joint+1}: {value:.4f}")

    print("\n---- RMS External Torque ----")
    for joint, value in enumerate(ext_rms_list):
        print(f"  Joint {joint+1}: {value:.4f}")


def main(folder_name):
    base_folder = '../data'
    folder_path = os.path.join(base_folder, folder_name)

    config_file = os.path.join(folder_path, 'config.txt')
    kinematics_file = os.path.join(folder_path, 'kinematics.txt')
    dynamics_file = os.path.join(folder_path, 'dynamics.txt')

    kinematics_vars, dynamics_vars = read_config(config_file)
    kinematics_data = read_data(kinematics_file, kinematics_vars, dof=4)
    dynamics_data = read_data(dynamics_file, dynamics_vars, dof=4)

    plot_all_vars(kinematics_data, dynamics_data, dof=4)
    calculate_errors(kinematics_data, dynamics_data, dof=4)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <folder_name>")
    else:
        main(sys.argv[1])
