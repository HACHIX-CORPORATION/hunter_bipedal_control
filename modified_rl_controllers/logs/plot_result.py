import numpy as np
import matplotlib.pyplot as plt
import os
from enum import IntEnum

LOG_DIR = f"{os.path.dirname(os.path.abspath(__file__))}/20251003_160559"
OBSERVATION_DATA_FILENAME = "observation_data.csv"
ACTION_DATA_FILENAME = "action_data.csv"
LOOP_RATE = 0.002

DEFAULT_JOINT_ANGLE = np.array([
    0.0,    # leg_l1_joint
    -0.0,   # leg_l2_joint
    -0.36,  # leg_l3_joint
    0.72,   # leg_l4_joint
    -0.36,  # leg_l5_joint
    0.0,    # leg_r1_joint
    -0.05,  # leg_r2_joint
    -0.36,  # leg_r3_joint
    0.72,   # leg_r4_joint
    -0.36   # leg_r5_joint
])

class ObservationIndex(IntEnum):
    LOOP_COUNT = 0
    BASEANGVEL_X = 1
    BASEANGVEL_Y = 2
    BASEANGVEL_Z = 3
    IMUZAXIS_X = 4
    IMUZAXIS_Y = 5
    IMUZAXIS_Z = 6
    DELTAJOINTPOS_0 = 7
    DELTAJOINTPOS_1 = 8
    DELTAJOINTPOS_2 = 9
    DELTAJOINTPOS_3 = 10
    DELTAJOINTPOS_4 = 11
    DELTAJOINTPOS_5 = 12
    DELTAJOINTPOS_6 = 13
    DELTAJOINTPOS_7 = 14
    DELTAJOINTPOS_8 = 15
    DELTAJOINTPOS_9 = 16
    JOINTVEL_0 = 17
    JOINTVEL_1 = 18
    JOINTVEL_2 = 19
    JOINTVEL_3 = 20
    JOINTVEL_4 = 21
    JOINTVEL_5 = 22
    JOINTVEL_6 = 23
    JOINTVEL_7 = 24
    JOINTVEL_8 = 25
    JOINTVEL_9 = 26
    LASTACTIONS_0 = 27
    LASTACTIONS_1 = 28
    LASTACTIONS_2 = 29
    LASTACTIONS_3 = 30
    LASTACTIONS_4 = 31
    LASTACTIONS_5 = 32
    LASTACTIONS_6 = 33
    LASTACTIONS_7 = 34
    LASTACTIONS_8 = 35
    LASTACTIONS_9 = 36
    COMMAND_X = 37
    COMMAND_Y = 38
    COMMAND_YAW = 39
    GAITFREQUENCY = 40
    GAIT = 41

class ActionIndex(IntEnum):
    LOOP_COUNT = 0
    ACTION_0 = 1
    ACTION_1 = 2
    ACTION_2 = 3
    ACTION_3 = 4
    ACTION_4 = 5
    ACTION_5 = 6
    ACTION_6 = 7
    ACTION_7 = 8
    ACTION_8 = 9
    ACTION_9 = 10

if __name__ == "__main__":
    # load observation data
    observation_data = np.loadtxt(f"{LOG_DIR}/{OBSERVATION_DATA_FILENAME}", delimiter=",", skiprows=1)
    
    # load action data
    action_data = np.loadtxt(f"{LOG_DIR}/{ACTION_DATA_FILENAME}", delimiter=",", skiprows=1)
    
    result_dir = f"{LOG_DIR}/graphs"
    os.makedirs(result_dir, exist_ok=True)

    joint_names_left = ['leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint']
    joint_names_right = ['leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint']

    print(f"Plots saved to {result_dir}/")
    # Create time axis based on loop count and control time step
    time_axis = observation_data[:, ObservationIndex.LOOP_COUNT] * LOOP_RATE
    
    # Figure 1: Base Angular Velocity (3 subplots)
    fig1, axes1 = plt.subplots(3, 1, figsize=(12, 10))
    fig1.suptitle('Observation: Base Angular Velocity', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)
    
    # Plot BASEANGVEL_X
    axes1[0].plot(time_axis, observation_data[:, ObservationIndex.BASEANGVEL_X], 'b-', linewidth=1)
    axes1[0].set_ylabel('Base Ang Vel X (rad/s)')
    axes1[0].grid(True, alpha=0.3)
    axes1[0].set_title('Base Angular Velocity X')
    
    # Plot BASEANGVEL_Y
    axes1[1].plot(time_axis, observation_data[:, ObservationIndex.BASEANGVEL_Y], 'g-', linewidth=1)
    axes1[1].set_ylabel('Base Ang Vel Y (rad/s)')
    axes1[1].grid(True, alpha=0.3)
    axes1[1].set_title('Base Angular Velocity Y')
    
    # Plot BASEANGVEL_Z
    axes1[2].plot(time_axis, observation_data[:, ObservationIndex.BASEANGVEL_Z], 'r-', linewidth=1)
    axes1[2].set_ylabel('Base Ang Vel Z (rad/s)')
    axes1[2].set_xlabel('Time (s)')
    axes1[2].grid(True, alpha=0.3)
    axes1[2].set_title('Base Angular Velocity Z')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/observation_base_angular_velocity.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- observation_base_angular_velocity.png")

    # Figure 2: IMU Z-axis (3 subplots)
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 10))
    fig2.suptitle('Observation: IMU Z-axis Data', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)
    
    # Plot IMUZAXIS_X
    axes2[0].plot(time_axis, observation_data[:, ObservationIndex.IMUZAXIS_X], 'b-', linewidth=1)
    axes2[0].set_ylabel('IMU Z-axis X')
    axes2[0].grid(True, alpha=0.3)
    axes2[0].set_title('IMU Z-axis X')
    
    # Plot IMUZAXIS_Y
    axes2[1].plot(time_axis, observation_data[:, ObservationIndex.IMUZAXIS_Y], 'g-', linewidth=1)
    axes2[1].set_ylabel('IMU Z-axis Y')
    axes2[1].grid(True, alpha=0.3)
    axes2[1].set_title('IMU Z-axis Y')
    
    # Plot IMUZAXIS_Z
    axes2[2].plot(time_axis, observation_data[:, ObservationIndex.IMUZAXIS_Z], 'r-', linewidth=1)
    axes2[2].set_ylabel('IMU Z-axis Z')
    axes2[2].set_xlabel('Time (s)')
    axes2[2].grid(True, alpha=0.3)
    axes2[2].set_title('IMU Z-axis Z')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/observation_imu_zaxis.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- observation_imu_zaxis.png")


    joint_pose = observation_data[:, ObservationIndex.DELTAJOINTPOS_0:ObservationIndex.DELTAJOINTPOS_9+1] + DEFAULT_JOINT_ANGLE

    # Figure 3: Left leg joint poses 
    fig3, axes3 = plt.subplots(5, 1, figsize=(12, 15))
    fig3.suptitle('Observation: Left Leg Joint Poses', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)

    for i in range(5):
        axes3[i].plot(time_axis, joint_pose[:, i], 'b-', linewidth=1)
        axes3[i].set_ylabel(f'{joint_names_left[i]} (rad)')
        axes3[i].grid(True, alpha=0.3)
        axes3[i].set_title(f'{joint_names_left[i]} (rad)')
        if i == 4:  # Last subplot
            axes3[i].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/observation_left_leg_joint_poses.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- observation_left_leg_joint_poses.png")

    # Figure 4: Right leg joint poses (joint_pose index 5->9)
    fig4, axes4 = plt.subplots(5, 1, figsize=(12, 15))
    fig4.suptitle('Observation: Right Leg Joint Poses', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)
    
    for i in range(5):
        axes4[i].plot(time_axis, joint_pose[:, i+5], 'r-', linewidth=1)
        axes4[i].set_ylabel(f'{joint_names_right[i]} (rad)')
        axes4[i].grid(True, alpha=0.3)
        axes4[i].set_title(f'{joint_names_right[i]} (rad)')
        if i == 4:  # Last subplot
            axes4[i].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/observation_right_leg_joint_poses.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- observation_right_leg_joint_poses.png") 

    # Figure 5: Left leg joint velocities
    fig5, axes5 = plt.subplots(5, 1, figsize=(12, 15))
    fig5.suptitle('Observation: Left Leg Joint Velocities', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)

    for i in range(5):
        axes5[i].plot(time_axis, observation_data[:, ObservationIndex.JOINTVEL_0 + i], 'b-', linewidth=1)
        axes5[i].set_ylabel(f'{joint_names_left[i]} (rad/s)')
        axes5[i].grid(True, alpha=0.3)
        axes5[i].set_title(f'{joint_names_left[i]} (rad/s)')
        if i == 4:  # Last subplot
            axes5[i].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/observation_left_leg_joint_velocities.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- observation_left_leg_joint_velocities.png")

    # Figure 6: Left leg last actions
    fig6, axes6 = plt.subplots(5, 1, figsize=(12, 15))
    fig6.suptitle('Observation: Left Leg Last Actions', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)

    for i in range(5):
        axes6[i].plot(time_axis, observation_data[:, ObservationIndex.LASTACTIONS_0 + i], 'b-', linewidth=1)
        axes6[i].set_ylabel(f'{joint_names_left[i]} (action)')
        axes6[i].grid(True, alpha=0.3)
        axes6[i].set_title(f'{joint_names_left[i]} (action)')
        if i == 4:  # Last subplot
            axes6[i].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/observation_left_leg_last_actions.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- observation_left_leg_last_actions.png")

    # Figure 7: Right leg last actions
    fig7, axes7 = plt.subplots(5, 1, figsize=(12, 15))
    fig7.suptitle('Observation: Right Leg Last Actions', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)

    for i in range(5):
        axes7[i].plot(time_axis, observation_data[:, ObservationIndex.LASTACTIONS_0 + i + 5], 'r-', linewidth=1)
        axes7[i].set_ylabel(f'{joint_names_right[i]} (action)')
        axes7[i].grid(True, alpha=0.3)
        axes7[i].set_title(f'{joint_names_right[i]} (action)')
        if i == 4:  # Last subplot
            axes7[i].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/observation_right_leg_last_actions.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- observation_right_leg_last_actions.png")

    # Figure 8: Command x, y, z in same figure
    fig8, axes8 = plt.subplots(3, 1, figsize=(12, 10))
    fig8.suptitle('Observation: Command Data', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)
    
    # Plot COMMAND_X
    axes8[0].plot(time_axis, observation_data[:, ObservationIndex.COMMAND_X], 'b-', linewidth=1)
    axes8[0].set_ylabel('Command X (m/s)')
    axes8[0].grid(True, alpha=0.3)
    axes8[0].set_title('Command X')
    
    # Plot COMMAND_Y
    axes8[1].plot(time_axis, observation_data[:, ObservationIndex.COMMAND_Y], 'g-', linewidth=1)
    axes8[1].set_ylabel('Command Y (m/s)')
    axes8[1].grid(True, alpha=0.3)
    axes8[1].set_title('Command Y')
    
    # Plot COMMAND_YAW
    axes8[2].plot(time_axis, observation_data[:, ObservationIndex.COMMAND_YAW], 'r-', linewidth=1)
    axes8[2].set_ylabel('Command Yaw (rad/s)')
    axes8[2].set_xlabel('Time (s)')
    axes8[2].grid(True, alpha=0.3)
    axes8[2].set_title('Command Yaw')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/observation_command_data.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- observation_command_data.png")

    # Figure 9: Gait frequency and gait type
    fig9, axes9 = plt.subplots(2, 1, figsize=(12, 8))
    fig9.suptitle('Observation: Gait Data', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)
    
    # Plot GAITFREQUENCY
    axes9[0].plot(time_axis, observation_data[:, ObservationIndex.GAITFREQUENCY], 'b-', linewidth=1)
    axes9[0].set_ylabel('Gait Frequency (Hz)')
    axes9[0].grid(True, alpha=0.3)
    axes9[0].set_title('Gait Frequency')
    
    # Plot GAIT
    axes9[1].plot(time_axis, observation_data[:, ObservationIndex.GAIT], 'g-', linewidth=1)
    axes9[1].set_ylabel('Gait Type')
    axes9[1].set_xlabel('Time (s)')
    axes9[1].grid(True, alpha=0.3)
    axes9[1].set_title('Gait Type')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/observation_gait_data.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- observation_gait_data.png")

    # Figure 10: Left leg action data (index 0-4)
    fig10, axes10 = plt.subplots(5, 1, figsize=(12, 15))
    fig10.suptitle('Action: Left Leg Actions', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)

    for i in range(5):
        axes10[i].plot(time_axis, action_data[:, ActionIndex.ACTION_0 + i], 'b-', linewidth=1)
        axes10[i].set_ylabel(f'{joint_names_left[i]} (action)')
        axes10[i].grid(True, alpha=0.3)
        axes10[i].set_title(f'{joint_names_left[i]} (action)')
        if i == 4:  # Last subplot
            axes10[i].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/action_left_leg.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- action_left_leg.png")

    # Figure 11: Right leg action data (index 5-9)
    fig11, axes11 = plt.subplots(5, 1, figsize=(12, 15))
    fig11.suptitle('Action: Right Leg Actions', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)

    for i in range(5):
        axes11[i].plot(time_axis, action_data[:, ActionIndex.ACTION_0 + i + 5], 'r-', linewidth=1)
        axes11[i].set_ylabel(f'{joint_names_right[i]} (action)')
        axes11[i].grid(True, alpha=0.3)
        axes11[i].set_title(f'{joint_names_right[i]} (action)')
        if i == 4:  # Last subplot
            axes11[i].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/action_right_leg.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- action_right_leg.png")

    target_joint_positions = action_data[:, ActionIndex.ACTION_0:ActionIndex.ACTION_9+1] + DEFAULT_JOINT_ANGLE

    # Figure 12: Target joint positios for left leg
    fig12, axes12 = plt.subplots(5, 1, figsize=(12, 15))
    fig12.suptitle('Action: Target Joint Positions for Left Leg', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)
    
    for i in range(5):
        axes12[i].plot(time_axis, target_joint_positions[:, i], 'b-', linewidth=1)
        axes12[i].set_ylabel(f'{joint_names_left[i]} (rad)')
        axes12[i].grid(True, alpha=0.3)
        axes12[i].set_title(f'{joint_names_left[i]} (rad)')
        if i == 4:  # Last subplot
            axes12[i].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/action_target_joint_positions_left_leg.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- action_target_joint_positions_left_leg.png")

    # Figure 13: Target joint 
    fig13, axes13 = plt.subplots(5, 1, figsize=(12, 15))
    fig13.suptitle('Action: Target Joint Positions for Right Leg', fontsize=14, y=1.05)
    plt.subplots_adjust(top=0.93)

    for i in range(5):
        axes13[i].plot(time_axis, target_joint_positions[:, i + 5], 'r-', linewidth=1)
        axes13[i].set_ylabel(f'{joint_names_right[i]} (rad)')
        axes13[i].grid(True, alpha=0.3)
        axes13[i].set_title(f'{joint_names_right[i]} (rad)')
        if i == 4:  # Last subplot
            axes13[i].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f"{result_dir}/action_target_joint_positions_right_leg.png", dpi=300, bbox_inches='tight')
    plt.close()
    print("- action_target_joint_positions_right_leg.png")