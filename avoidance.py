import numpy as np
#from robomaster import robot
#from robomaster import camera
from matplotlib import pyplot as plt
import time
import math 
#import detection
#import gripping
#import cv2

max_v = 1.0
min_v = -0.5
resolution_v = 0.05
max_yaw = 40.0*math.pi/180.0
resolution_yaw = 2.0 * math.pi/180.0
max_accel = 0.2
max_yaw_accel = 40.0 * math.pi / 180.0 # speed parameters

dt = 0.1
predict_time = 2.0 # time to predict over

speed_cost_gain = 1.0
obstacle_cost_gain = 1.0
to_goal_cost_gain = 0.15 # params for cost function

robot_width = 0.4
robot_length = 0.4 # dimensions for collision check

def dwa_control(x, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x)

    u, trajectory = calc_control_and_trajectory(x, dw, goal, ob)

    return u, trajectory


# calculate where robot
# will be based on control parameters
# x (state) will be [x, y, heading, vx, vheading]
def motion(x, u, dt):
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]
    return x

def calc_dynamic_window(x):
    """
    calculation dynamic window based on current state x
    """
    # calculating dynamic window approach
    
    # Dynamic window from robot specification
    Vs = [min_v, max_v, -max_yaw, max_yaw]
    
    # Dynamic window from motion model
    Vd = [x[3] - max_accel * dt,
          x[3] + max_accel * dt,
          x[4] - max_yaw_accel * dt,
          x[4] + max_yaw_accel * dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw

def predict_trajectory(x_init, v, y):
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= predict_time:
        x = motion(x, [v, y], dt)
        trajectory = np.vstack((trajectory, x))
        time += dt

    return trajectory

def calc_control_and_trajectory(x, dw, goal, ob):

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], resolution_v):
        for y in np.arange(dw[2], dw[3], resolution_yaw):

            trajectory = predict_trajectory(x_init, v, y)
            # calc cost
            to_goal_cost = to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = speed_cost_gain * (max_v - trajectory[-1, 3])
            ob_cost = obstacle_cost_gain * calc_obstacle_cost(trajectory, ob)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < 0.0 and abs(x[3]) < 0.0:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -max_yaw
    return best_u, best_trajectory

def calc_obstacle_cost(trajectory, ob):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    yaw = trajectory[:, 2]
    rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rot = np.transpose(rot, [2, 0, 1])
    local_ob = ob[:, None] - trajectory[:, 0:2]
    local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    local_ob = np.array([local_ob @ x for x in rot])
    local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    upper_check = local_ob[:, 0] <= robot_length / 2
    right_check = local_ob[:, 1] <= robot_width / 2
    bottom_check = local_ob[:, 0] >= -robot_length / 2
    left_check = local_ob[:, 1] >= -robot_width / 2
    if (np.logical_and(np.logical_and(upper_check, right_check),
                       np.logical_and(bottom_check, left_check))).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK

def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost

# plotting functions for debugging
def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw):  # pragma: no cover
    outline = np.array([[-robot_length / 2, robot_length / 2,
                       (robot_length / 2), -robot_length / 2,
                        -robot_length / 2],
                        [robot_width / 2, robot_width / 2,
                        -robot_width / 2, -robot_width / 2,
                        robot_width / 2]])
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                    [-math.sin(yaw), math.cos(yaw)]])
    outline = (outline.T.dot(Rot1)).T
    outline[0, :] += x
    outline[1, :] += y
    plt.plot(np.array(outline[0, :]).flatten(),
            np.array(outline[1, :]).flatten(), "-k")

def main(gx=3.0, gy=3.0):
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])

    show_animation = True

    # input [forward speed, yaw_rate]

    trajectory = np.array(x)

    # obstacles for now will be preset
    boxes_locations = [[1,1],[1,2],[2,1],[2,2]]
    ob = []
    for i in boxes_locations:
        a = i[0]
        b = i[1]
        ob.append([a-0.13, b-0.13])
        ob.append([a+0.13, b-0.13])
        ob.append([a+0.13, b+0.13])
        ob.append([a-0.13, b+0.13])
        
    ob = np.array(ob)
    
    while True:
        u, predicted_trajectory = dwa_control(x, goal, ob)
        u[0] = u[0]*-1
        u[1] = u[1]*-1
        x = motion(x, u, dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(x[0], x[1], x[2])
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= 0.2:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)
        plt.show()



if __name__ == '__main__':
    main()
    '''
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    '''
