import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D


#六维台参数定义
#上下平台尺寸，铰点分布角，两平台距离，铰点自由度，杆尺寸
R_up = 30
R_DP = 50
theta_up = 15
theta_dp = 18
high = 120

roll = 30
pitch = 30
yaw = 20
location  = [20,40,130] #上平台中心目标位置

#铰点相对坐标计算函数
def point_compute(r,theta,z):
    if theta>90 and theta < 270:
        flag_x = -1
    else:
        flag_x = 1
    if theta>0 and theta < 180:
        flag_y = 1
    else:
        flag_y = -1

    theta_compute = theta
    while(theta_compute>90):
        theta_compute -= 90
    theta_compute = math.radians(theta_compute)
    tan = math.tan(theta_compute)    
    x = flag_x*r/(math.sqrt(tan**2+1))
    y = flag_y*abs(x)*tan
    if (theta >90 and theta<180) or (theta>270 and theta<360):
        y = flag_y*r/(math.sqrt(tan**2+1))
        x = flag_x*abs(x)*tan
    return [x,y,0]


#输入：上下平台半径，分布角，上平台高度
#输出：各铰点相对坐标
def location_compute(r_up,t_up,r_dp,t_dp,high):
    DP1 = point_compute(r_dp,210+t_dp,0)
    DP6 = point_compute(r_dp,210-t_dp,0)
    DP3 = point_compute(r_dp,330+t_dp,0)
    DP2 = point_compute(r_dp,330-t_dp,0)
    DP5 = point_compute(r_dp,90+t_dp,0)
    DP4 = point_compute(r_dp,90-t_dp,0)
    DP = [DP1,DP2,DP3,DP4,DP5,DP6]
    UP2 = point_compute(r_up,270+t_up,0)
    UP1 = point_compute(r_up,270-t_up,0)
    UP4 = point_compute(r_up,30+t_up,0)
    UP3 = point_compute(r_up,30-t_up,0)
    UP6 = point_compute(r_up,150+t_up,0)
    UP5 = point_compute(r_up,150-t_up,0)
    UP = [UP1,UP2,UP3,UP4,UP5,UP6]
    return [DP,UP]


#相对坐标生成
DP,UP_origin =  location_compute(R_up,theta_up,R_DP,theta_dp,high)


#运动学反解函数
#输入：目标位姿，各铰点相对坐标
#输出；上平台圆形，上平台各点坐标，各支链杆长
def inv_solution(roll,pitch,yaw,location,UP_origin,DP):# 根据欧拉角顺序 Yaw -> Pitch -> Roll 计算总的旋转矩阵
    
    def get_combined_rotation_matrix(roll, pitch, yaw):
        roll = math.radians(roll)   # 滚转角：30°
        pitch = math.radians(pitch)  # 俯仰角：45°
        yaw = math.radians(yaw)    # 偏航角：60°
        """结合偏航、俯仰和滚转角的旋转矩阵，按Yaw -> Pitch -> Roll顺序"""
        rotation_matrix = np.array([
            [math.cos(yaw) * math.cos(pitch), 
            math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll), 
            math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.cos(roll)],
            
            [math.sin(yaw) * math.cos(pitch), 
            math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll), 
            math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)],
            
            [-math.sin(pitch), 
            math.cos(pitch) * math.sin(roll), 
            math.cos(pitch) * math.cos(roll)]
        ])
        return rotation_matrix

    # 创建圆的数据
    theta = np.linspace(0, 2 * np.pi, 100)
    circle_x2 = 30 * np.cos(theta)
    circle_y2 = 30 * np.sin(theta)
    circle_z2 = np.full_like(circle_x2, 0)  # Z坐标是常数等于'high'

    # 获取旋转矩阵
    R = get_combined_rotation_matrix(roll,pitch,yaw)
    rotated_points2 = []
    rotated_circle_points = []
    for i in range(len(circle_x2)):
        point = np.array([circle_x2[i], circle_y2[i], circle_z2[i]])
        rotated_point = R @ point + location  # 旋转后，保持圆心不变
        rotated_circle_points.append(rotated_point)

    rotated_points2 = np.array(rotated_circle_points)
    UP = []
    for up in UP_origin:
        UP.append(R @ up + location)
        
    def lenth(x,y):
        x1 = x[0]
        x2 = y[0]
        y1 = x[1]
        y2 = y[1]
        z1 = x[2]
        z2 = y[2]
        return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)

    L_set = []
    inv_ans = zip(UP,DP)
    for x,y in inv_ans:
        L_set.append(lenth(x,y))
    
    
    
    return [rotated_points2,UP,L_set]

#下平台圆周计算
def domn_plat():
    theta = np.linspace(0, 2 * np.pi, 100)
    radius = 50
    circle_x = radius * np.cos(theta)
    circle_y = radius * np.sin(theta)
    circle_z = np.zeros_like(circle_x)  # 在XY平面上
        # 对圆的每个点应用旋转矩阵
    rotated_points = []
    for i in range(len(circle_x)):
        point = np.array([circle_x[i], circle_y[i], circle_z[i]])
        rotated_point = point  
        rotated_points.append(rotated_point)
    rotated_points = np.array(rotated_points)
    return rotated_points

#生成上下平台
rotated_points = domn_plat()
rotated_points2,UP,L_set = inv_solution(roll,pitch,yaw,location,UP_origin,DP)

#打印反解结果
print(f"UP:{UP}")
print(f"DP:{DP}")
for i in range(len(L_set)):
    print(f"杆长L{i+1}:{L_set[i]}")




# 数据点 (Down 和 Up)
down_points = np.array(DP)
up_points = np.array(UP)
# 绘制图形
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')
# 绘制下平台点（红色）和上平台点（蓝色）
ax.scatter(down_points[:, 0], down_points[:, 1], down_points[:, 2], color='red', label='Down points')
ax.scatter(up_points[:, 0], up_points[:, 1], up_points[:, 2], color='blue', label='Up points')
# 标注上平台（UP）和下平台（DP）的编号
for i, (dp_point, up_point) in enumerate(zip(down_points, up_points)):
    ax.text(dp_point[0], dp_point[1], dp_point[2], f'DP{i+1}', color='red', fontsize=12)
    ax.text(up_point[0], up_point[1], up_point[2], f'UP{i+1}', color='blue', fontsize=12)
# 绘制第一个圆（XY平面，旋转后）
ax.plot(rotated_points[:, 0], rotated_points[:, 1], rotated_points[:, 2], color='green', label='Rotated Circle with radius 50')
# 绘制第二个圆（XY平面，旋转后）
ax.plot(rotated_points2[:, 0], rotated_points2[:, 1], rotated_points2[:, 2], color='purple', label=f'Rotated Circle with radius 30 at  {location})')
# 连接DP和UP_origin中的点的直线
for dp_point, up_point in zip(down_points, up_points):
    ax.plot([dp_point[0], up_point[0]], [dp_point[1], up_point[1]], [dp_point[2], up_point[2]], color='black')
# 设置标签和标题
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title(f'3D Plot with Rotated Circles and Connecting Lines\n(Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°)')
# 显示图例
ax.legend()
# 设置等比例显示
ax.set_box_aspect([1, 1, 1])  # 等比例设置
plt.show()




