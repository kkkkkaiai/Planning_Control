import numpy as np
import matplotlib.pyplot as plt

# use circle and arrow to represent the robot
# the circle is the robot
import matplotlib.patches as patches
# use the arrow to represent the robot
import matplotlib.transforms as transforms
import matplotlib.patches as mpatches

robot_direction = 0
robot_position = np.array([0, 0, robot_direction])

target_position = np.array([0.3, 2])
target_direction = np.pi / 2

robot_radius = 0.1
robot_arrow_length = 0.2
robot_arrow_width = 0.02

# no filled 
robot_patch = patches.Circle(robot_position[0:2], radius=robot_radius, color='black', fill=False)
robot_arrow = patches.FancyArrow(robot_position[0], robot_position[1], robot_radius * np.cos(robot_direction), robot_radius * np.sin(robot_direction), width=robot_arrow_width, color='black')

# target patch is a rectangle with direction
target_patch_length = 0.2
target_patch_width = 0.01
target_patch = patches.Rectangle(target_position, target_patch_length, target_patch_width, angle=(target_direction-np.pi/2) * 180 / np.pi, color='red')
# target arrow is a arrow with direction
dx = 0.2 * np.cos(target_direction)
dy = 0.2 * np.sin(target_direction) 
# get the center of the rectangle
arrow_position = target_position + np.array([target_patch_length/2 * np.sin(target_direction), target_patch_width/2 * np.sin(target_direction)])
# the arrow is from the center of the rectangle
target_arrow = patches.FancyArrow(arrow_position[0], arrow_position[1], dx, dy, width=0.02, color='red')

# diff robot kinematics 
def kin(x, input, dt):
    # x: [x, y, theta]
    # input: [v, w]
    v = input[0]
    w = input[1]
    theta = x[2]
    x_next = np.zeros(3)
    x_next[0] = x[0] + v * np.cos(theta) * dt
    x_next[1] = x[1] + v * np.sin(theta) * dt
    x_next[2] = x[2] + w * dt
    return x_next

class direction_pid:
    def __init__(self) -> None:
        self.reset()

    def set_target(self, state_target, direction_target):
        self.state_target = state_target
        self.direction_target = direction_target

    def reset(self):
        self.p_w = 5
        self.i_w = 0
        self.d_w = 0.0
        self.target = None
        self.last_error = None
        self.error_sum = 0

        self.w1 = 1
        self.w2 = 0

    def run(self, state, direction):
        if self.state_target is None or self.direction_target is None:
            raise Exception('Target is not set')
        
        error1 = self.w1*(self.state_target - state)
        error2 = self.w2*(self.direction_target - direction)
        error = error1 + error2
        print(error1, error2)
        if self.last_error is not None:
            diff_error = error - self.last_error
        else:
            diff_error = 0

        self.last_error = error
        self.error_sum += error

        w = self.p_w * error + self.i_w * self.error_sum + self.d_w * diff_error

        return w
    
class position_pid:
    def __init__(self) -> None:
        self.reset()

    def set_target(self, target):
        self.target = target

    def reset(self):
        self.p_w = 0.5
        self.i_w = 0.0
        self.d_w = 0.1
        self.target = None
        self.last_error = None
        self.error_sum = 0

    def run(self, state):
        if self.target is None:
            raise Exception('Target is not set')
        
        target = self.target
        # calculate the distance between the target and the robot
        error = np.linalg.norm(target - state)

        if self.last_error is not None:
            diff_error = error - self.last_error
        else:
            diff_error = 0

        self.last_error = error
        self.error_sum += error

        v = self.p_w * error + self.i_w * self.error_sum + self.d_w * diff_error

        return v

# plot the robot
ax = plt.gca()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
# change the figure size
ax.add_patch(robot_patch)
ax.add_patch(robot_arrow)
ax.add_patch(target_patch)
ax.add_patch(target_arrow)
plt.axis('equal')
iteration = 100
dt = 0.1


direction_pid = direction_pid()
direction_pid.set_target(0, target_direction)
position_pid = position_pid()
position_pid.set_target(arrow_position)

# construct a line function to calculate the distance
line_function = lambda x: np.tan(target_direction) * x + (target_position[1] - np.tan(target_direction) * target_position[0])
# calculate the distance from a point to the line
distance_function = lambda x, y: np.abs(np.tan(target_direction) * x - y + target_position[1] - np.tan(target_direction) * target_position[0]) / np.sqrt(np.tan(target_direction) ** 2 + 1)

# the relative transform of the target based on the robot coordinate rather than the world coordinate
while(iteration):
    iteration -= 1
    # calculate the relative position and direction

    robot_transform_matraix = np.array([[np.cos(robot_position[2]), -np.sin(robot_position[2]), robot_position[0]], [np.sin(robot_position[2]), np.cos(robot_position[2]), robot_position[1]], [0, 0, 1]])
    target_transform_matrix = np.array([[np.cos(target_direction), -np.sin(target_direction), arrow_position[0]], [np.sin(target_direction), np.cos(target_direction), arrow_position[1]], [0, 0, 1]])
    # calculate the relative transform matrix based on the robot coordinate
    relative_transform_matrix = np.linalg.inv(robot_transform_matraix) @ target_transform_matrix

    relative_x = relative_transform_matrix[0, 2]
    relative_y = relative_transform_matrix[1, 2]
    
    # get the input
    w = direction_pid.run(relative_y, robot_position[2])
    v = position_pid.run(robot_position[0:2])

    # check the error of direction
    if np.abs(relative_y) < 0.1:

    input = np.array([v, w])

    # get the next state
    robot_position = kin(robot_position, input, dt)
    # update the robot patch
    robot_patch.set_center(robot_position[0:2])
    dx = robot_arrow_length * np.cos(robot_position[2])
    dy = robot_arrow_length * np.sin(robot_position[2])
    robot_arrow.set_data(x=robot_position[0], y=robot_position[1], dx=dx, dy=dy)
    
    # plot the robot
    plt.pause(1)


plt.show()