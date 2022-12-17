#!/usr/bin/python3

# This code takes 4 coordinates (belonging to the wheel base)
# and gives corrections as described by the paper

import numpy as np

def normalize_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi

def main():
    data = np.loadtxt('square_test_data', delimiter='\t')

    side = 1.5

    del_x = [0 for i in range(6)]
    del_y = [0 for i in range(6)]
    del_tht = [0 for i in range(6)]

    for i in range(6):
        x1 = data[i, 0]
        y1 = data[i, 1]
        x2 = data[i, 2]
        y2 = data[i, 3]
        x3 = data[i, 4]
        y3 = data[i, 5]
        x4 = data[i, 6]
        y4 = data[i, 7]

        # x1 = -0.866 + 1
        # y1 = -0.5 + 2
        # x2 = 0.866 + 1
        # y2 = 0.5 + 2
        # x3 = -1 + 1
        # y3 = 0 + 2
        # x4 = 1 + 1
        # y4 = 0 + 2

        tht1 = np.arctan2(y2 - y1, x2 - x1)
        tht2 = np.arctan2(y4 - y3, x4 - x3)

        shift_x = (x1 + x2) / 2
        shift_y = (y1 + y2) / 2

        x1 -= shift_x
        x2 -= shift_x
        x3 -= shift_x
        x4 -= shift_x

        y1 -= shift_y
        y2 -= shift_y
        y3 -= shift_y
        y4 -= shift_y

        left_i = np.array([[x1], [y1]])
        left_f = np.array([[x3], [y3]])
        right_i = np.array([[x2], [y2]])
        right_f = np.array([[x4], [y4]])

        rot_matrix = np.array([[np.cos(tht1), np.sin(tht1)],
                               [-np.sin(tht1), np.cos(tht1)]])

        tf_left_i = rot_matrix @ left_i
        tf_left_f = rot_matrix @ left_f
        tf_right_i = rot_matrix @ right_i
        tf_right_f = rot_matrix @ right_f

        del_xy = (tf_right_f + tf_left_f) / 2
        del_x[i] = del_xy[0]
        del_y[i] = del_xy[1]

        tht1 = normalize_angle(tht1)
        tht2 = normalize_angle(tht2)
        del_tht[i] = tht2 - tht1

        # print(f"\n initial tf pose \n\n Left: \n {tf_left_i} \n\n Right:\n {tf_right_i} \n\n")
        # print("Deltas", del_x[i], del_y[i], (del_tht[i] * 360 / (2 * np.pi)))


    avg_del_x_acw = sum(del_x[0:3])/3
    avg_del_y_acw = sum(del_y[0:3])/3
    avg_del_x_cw = sum(del_x[3:6])/3
    avg_del_y_cw = sum(del_y[3:6])/3

    print(avg_del_x_cw, avg_del_x_acw, avg_del_y_cw, avg_del_y_acw )

    alpha_y = (avg_del_y_cw-avg_del_y_acw)/(-4*side)
    alpha_x = (avg_del_x_cw+avg_del_x_acw)/(-4*side)
    beta_y = (avg_del_y_cw+avg_del_y_acw)/(-4*side)
    beta_x = (avg_del_x_cw-avg_del_x_acw)/(-4*side)

    # 2 *(0.805724) y_acw
    # 2*(0.787273) y_cw

if __name__ == "__main__":
    main()
