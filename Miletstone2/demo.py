import numpy as np
from FrankaRobot16384 import Franka16384       # Robot wrapper
from milestone_2 import Robot                  # Student motion planner
from frankapy.utils import min_jerk            # Example interpolation

# --------------------------- Helper ---------------------------
def wait_for_user(prompt="[Press 'n' for next, 'r' to retry, or 'q' to quit]: "):
    """Pause execution and wait for valid user input."""
    while True:
        user_input = input(prompt).strip().lower()
        if user_input in ['n', 'r', 'q']:
            return user_input
        print("Invalid input. Please press 'n', 'r', or 'q'.")



def main():

    franka = Franka16384()  
    planner = Robot()       


    franka.reset()
    q_curr = franka.get_joints()  

    # Define targets
    q_pick = np.array([0.02539866,  0.13048301, -0.06443245, -2.08149549,
                       0.01439907,  2.09092226,  0.74839636])
    q_place = np.array([-0.0341,  0.2156, -0.0854, -2.2618, 0.0314, 2.1323, 0.9871])

    T = 2.0
    dt = 0.02


    print("\n[Step 1] Moving to pick location...")
    traj_pick = [min_jerk(q_curr, q_pick, t, T) for t in np.arange(0, T, dt)]
    # TODO: Replace min-jerk with student’s trapezoidal trajectory function

    franka.follow_trajectory(traj_pick, T=T)

    # Standby for user input before next action
    while True:
        user_choice = wait_for_user("Press 'n' to continue to pick or 'r' to retry move: ")
        if user_choice == 'n':
            break
        elif user_choice == 'r':
            print("[Retrying move to pick pose...]")
            franka.follow_trajectory(traj_pick, T=T)
        elif user_choice == 'q':
            print("[Exiting program]")
            return

    # ---------------- Step 2: Pick Action ---------------------
    print("\n[Step 2] Closing gripper to pick object...")
    franka.close_gripper()

    # Standby before next action
    while True:
        user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry pick: ")
        if user_choice == 'n':
            break
        elif user_choice == 'r':
            print("[Retrying pick action...]")
            franka.close_gripper()
        elif user_choice == 'q':
            print("[Exiting program]")
            return

    # ---------------- Step 3: Move to Place Pose ----------------
    print("\n[Step 3] Moving to place location...")
    traj_place = [min_jerk(q_pick, q_place, t, T) for t in np.arange(0, T, dt)]
    # TODO: Replace min-jerk with student’s trapezoidal trajectory function

    franka.follow_trajectory(traj_place, T=T)

    while True:
        user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry move: ")
        if user_choice == 'n':
            break
        elif user_choice == 'r':
            print("[Retrying move to place pose...]")
            franka.follow_trajectory(traj_place, T=T)
        elif user_choice == 'q':
            print("[Exiting program]")
            return

    # ---------------- Step 4: Place Action ---------------------
    print("\n[Step 4] Opening gripper to release object...")
    franka.open_gripper()

    while True:
        user_choice = wait_for_user("Press 'n' to finish or 'r' to retry place: ")
        if user_choice == 'n':
            break
        elif user_choice == 'r':
            print("[Retrying place action...]")
            franka.open_gripper()
        elif user_choice == 'q':
            print("[Exiting program]")
            return

    # ---------------- Step 5: Return Home ----------------------
    print("\n[Step 5] Returning to home position...")
    franka.reset()
    print("[Demo complete]")


if __name__ == "__main__":
    main()
Z