
# This is a python script to calculate the tangent points on a circle from an external point.

import matplotlib.pyplot as plt
import math
import numpy as np

def get_tangent_line(tx, ty, ox, oy, r):
    distance = math.sqrt((tx - ox) ** 2 + (ty - oy) ** 2)
    length = math.sqrt(distance ** 2 - r ** 2)

    if distance <= r:
        print("输入的数值不在范围内")
        return

    cx = abs(ox - tx) / distance
    cy = abs(oy - ty) / distance

    angle = math.asin(r / distance)

    q1x = cx * math.cos(angle) - cy * math.sin(angle)
    q1y = cx * math.sin(angle) + cy * math.cos(angle)
    q2x = cx * math.cos(-angle) - cy * math.sin(-angle)
    q2y = cx * math.sin(-angle) + cy * math.cos(-angle)

    q1x = q1x * length
    q1y = q1y * length
    q2x = q2x * length
    q2y = q2y * length

    if ox < tx:
        q1x = -q1x 
        q2x = -q2x
    
    if oy < ty:
        q1y = -q1y 
        q2y = -q2y

    q1x += tx
    q2x += tx
    q1y += ty
    q2y += ty

    return [q1x, q1y, q2x, q2y]

# 示例用法
circle_center = (0, 0)
circle_radius = 1.0
external_point = (-2, -2)

tangent_points = get_tangent_line(*external_point, *circle_center, circle_radius)

# 绘图
theta = np.linspace(0, 2 * np.pi, 100)
circle_x = circle_center[0] + circle_radius * np.cos(theta)
circle_y = circle_center[1] + circle_radius * np.sin(theta)

plt.figure(figsize=(8, 8))
plt.plot(circle_x, circle_y, label='Circle')
plt.scatter(*circle_center, color='red', marker='o', label='Circle Center')
plt.scatter(*external_point, color='blue', marker='o', label='External Point')
plt.scatter(tangent_points[0], tangent_points[1], color='green', marker='o', label='Tangent Point 1')
plt.scatter(tangent_points[2], tangent_points[3], color='purple', marker='o', label='Tangent Point 2')

# 绘制切线
plt.plot([external_point[0], tangent_points[0]], [external_point[1], tangent_points[1]], '--', color='orange', label='Tangent Line 1')
plt.plot([external_point[0], tangent_points[2]], [external_point[1], tangent_points[3]], '--', color='brown', label='Tangent Line 2')

# 设置图形参数
plt.xlim(-1.5, 3)
plt.ylim(-1.5, 3)
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.grid(color='gray', linestyle='--', linewidth=0.5)

# 添加标签和图例
plt.title('Tangent Line to Circle from External Point')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()

# 显示图形
plt.show()
