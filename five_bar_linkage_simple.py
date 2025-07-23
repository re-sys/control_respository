#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import math

class FiveBarLinkage:
    def __init__(self, link_lengths):
        """
        五连杆机构正运动学计算
        link_lengths: [l1, l2, l3, l4, l5] 各连杆长度
        l1: 固定连杆长度 (A到E)
        l2: 输入连杆1长度 (A到B)
        l3: 中间连杆长度 (B到C)
        l4: 输入连杆2长度 (E到D)
        l5: 中间连杆长度 (C到D)
        """
        self.l1, self.l2 = 0.2,0.16
        self.last_point = None
        
    def forward_kinematics(self, theta1, theta2):
        """
        五连杆正运动学计算
        theta1: 输入角度1 (弧度)
        theta2: 输入角度2 (弧度)
        返回: 各关节的坐标 [(x1,y1), (x2,y2), (x3,y3), (x4,y4), (x5,y5)]
        """
        # 固定关节A和E的坐标
        A = (0, 0)  # 关节1
        E = (0, 0)  # 关节5
        
        # 计算关节B的坐标 (连杆2的末端)
        B = (self.l1 * np.cos(theta1), self.l1 * np.sin(theta1))
        
        # 计算关节D的坐标 (连杆4的末端)
        D = (self.l1 * np.cos(theta2), self.l1 * np.sin(theta2))
        
        # 计算关节C的坐标 (连杆3和连杆5的交点)
        C = self._solve_joint_c(B, D)
        
        return [A, B, C, D, E]
    
    def _solve_joint_c(self, B, D):
        """
        求解关节C的坐标
        使用连杆3和连杆5的几何约束
        """
        Bx, By = B
        Dx, Dy = D
        
        # 连杆3的长度约束: |C-B| = l3
        # 连杆5的长度约束: |C-D| = l5
        
        # 使用解析方法求解
        # 设C = (x, y)，则有两个方程：
        # (x - Bx)² + (y - By)² = l3²
        # (x - Dx)² + (y - Dy)² = l5²
        
        # 展开并相减得到线性方程：
        # 2*(Dx-Bx)*x + 2*(Dy-By)*y = l3² - l5² + Dx² + Dy² - Bx² - By²
        
        # 系数
        a = 2 * (Dx - Bx)
        b = 2 * (Dy - By)
        c =  Dx**2 + Dy**2 - Bx**2 - By**2
        
        # 检查是否有解
        # 求解线性方程
        
            # 从线性方程解出y关于x的表达式
            # y = (c - a*x) / b
            # 代入第一个约束方程
            # (x - Bx)² + ((c - a*x)/b - By)² = l3²
            
        # 展开得到二次方程
        A_coeff = b**2 + (a)**2
        B_coeff = -2*Bx*(b**2) - 2*a*(c - b*By)
        C_coeff = (Bx**2)*b**2 + ((c - b*By))**2 - (self.l2**2)*b**2
        
        # 求解二次方程
        discriminant = B_coeff**2 - 4*A_coeff*C_coeff
        
        if discriminant < 0:
            # 无实数解，返回中点
            return ((Bx + Dx)/2, (By + Dy)/2)
        
        # 计算x的两个解
        x1 = (-B_coeff + np.sqrt(discriminant)) / (2*A_coeff)
        x2 = (-B_coeff - np.sqrt(discriminant)) / (2*A_coeff)
        
        # 计算对应的y值
        y1 = (c - a*x1) / b
        y2 = (c - a*x2) / b
        
        # 选择更合理的解（距离B更近的）
        point1 = np.array([x1, y1])
        point2 = np.array([x2, y2])
        dist1 = np.linalg.norm(point1)
        dist2 = np.linalg.norm(point2)
        if self.last_point is None:
            if dist1 >= dist2:
                self.last_point = point1
                return (x1, y1)
            else:
                self.last_point = point2
                return (x2, y2)
        else:
            if np.linalg.norm(point1-self.last_point) < np.linalg.norm(point2-self.last_point):
                self.last_point = point1
                return (x1, y1)
            else:
                self.last_point = point2
                return (x2, y2)
        

class FiveBarSimulator:
    def __init__(self):
        # 创建图形界面
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        plt.subplots_adjust(left=0.1, bottom=0.25, right=0.9, top=0.9)
        
        # 创建两个五连杆机构
        # 连杆长度: [l1, l2, l3, l4, l5]
        self.linkage1 = FiveBarLinkage([1.6, 2.0])  # 第一个五连杆
        self.linkage2 = FiveBarLinkage([1.6, 2.0])  # 第二个五连杆
        
        # 初始角度 (弧度)
        self.theta1_1 = 0.5  # 第一个机构的theta1
        self.theta2_1 = 2.0  # 第一个机构的theta2
        self.theta1_2 = 1.0  # 第二个机构的theta1
        self.theta2_2 = 2.5  # 第二个机构的theta2
        
        # 初始化绘图
        self.init_plot()
        self.create_widgets()
        self.update_plot()
        
    def init_plot(self):
        """初始化绘图"""
        self.ax.clear()
        self.ax.set_xlim(-1, 6)
        self.ax.set_ylim(-2, 3)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('五连杆机构仿真 - 正运动学可视化', fontsize=16, fontweight='bold')
        self.ax.set_xlabel('X轴')
        self.ax.set_ylabel('Y轴')
        
        # 初始化连杆线条
        self.lines1 = []  # 第一个机构的连杆
        self.lines2 = []  # 第二个机构的连杆
        self.joints1 = []  # 第一个机构的关节
        self.joints2 = []  # 第二个机构的关节
        
        # 创建连杆线条
        colors1 = ['blue', 'darkblue', 'navy', 'royalblue']
        colors2 = ['red', 'darkred', 'crimson', 'firebrick']
        
        for i in range(4):  # 4条连杆
            line1, = self.ax.plot([], [], color=colors1[i], linewidth=3, alpha=0.8, 
                                 label=f'机构1-连杆{i+1}' if i == 0 else "")
            line2, = self.ax.plot([], [], color=colors2[i], linewidth=3, alpha=0.8, 
                                 label=f'机构2-连杆{i+1}' if i == 0 else "")
            self.lines1.append(line1)
            self.lines2.append(line2)
        
        # 创建关节点
        joint_colors = ['black', 'blue', 'green', 'orange', 'purple']
        for i in range(5):  # 5个关节
            joint1, = self.ax.plot([], [], 'o', color=joint_colors[i], markersize=10, 
                                  label=f'关节{i+1}' if i == 0 else "")
            joint2, = self.ax.plot([], [], 'o', color=joint_colors[i], markersize=10, 
                                  label=f'关节{i+1}' if i == 0 else "")
            self.joints1.append(joint1)
            self.joints2.append(joint2)
        
        # 添加图例
        self.ax.legend(loc='upper right', fontsize=10)
        
    def create_widgets(self):
        """创建控制滑块"""
        # 第一个机构的控制滑块
        ax_theta1_1 = plt.axes([0.1, 0.15, 0.35, 0.03])
        ax_theta2_1 = plt.axes([0.1, 0.10, 0.35, 0.03])
        
        # 第二个机构的控制滑块
        ax_theta1_2 = plt.axes([0.55, 0.15, 0.35, 0.03])
        ax_theta2_2 = plt.axes([0.55, 0.10, 0.35, 0.03])
        
        # 创建滑块
        self.s_theta1_1 = Slider(ax_theta1_1, '机构1 θ1', 0, 2*np.pi, valinit=self.theta1_1)
        self.s_theta2_1 = Slider(ax_theta2_1, '机构1 θ2', 0, 2*np.pi, valinit=self.theta2_1)
        self.s_theta1_2 = Slider(ax_theta1_2, '机构2 θ1', 0, 2*np.pi, valinit=self.theta1_2)
        self.s_theta2_2 = Slider(ax_theta2_2, '机构2 θ2', 0, 2*np.pi, valinit=self.theta2_2)
        
        # 绑定滑块事件
        self.s_theta1_1.on_changed(self.update_angles)
        self.s_theta2_1.on_changed(self.update_angles)
        self.s_theta1_2.on_changed(self.update_angles)
        self.s_theta2_2.on_changed(self.update_angles)
        
        # 添加重置按钮
        ax_reset = plt.axes([0.8, 0.02, 0.1, 0.04])
        self.btn_reset = Button(ax_reset, '重置角度')
        self.btn_reset.on_clicked(self.reset_angles)
        
        # 添加动画按钮
        ax_animate = plt.axes([0.65, 0.02, 0.1, 0.04])
        self.btn_animate = Button(ax_animate, '动画演示')
        self.btn_animate.on_clicked(self.toggle_animation)
        
        self.animation_running = False
        
    def update_angles(self, val):
        """更新角度值"""
        self.theta1_1 = self.s_theta1_1.val
        self.theta2_1 = self.s_theta2_1.val
        self.theta1_2 = self.s_theta1_2.val
        self.theta2_2 = self.s_theta2_2.val
        self.update_plot()
        
    def reset_angles(self, event):
        """重置角度"""
        self.theta1_1 = 0.5
        self.theta2_1 = 2.0
        self.theta1_2 = 1.0
        self.theta2_2 = 2.5
        
        self.s_theta1_1.set_val(self.theta1_1)
        self.s_theta2_1.set_val(self.theta2_1)
        self.s_theta1_2.set_val(self.theta1_2)
        self.s_theta2_2.set_val(self.theta2_2)
        
    def toggle_animation(self, event):
        """切换动画状态"""
        self.animation_running = not self.animation_running
        if self.animation_running:
            self.btn_animate.label.set_text('停止动画')
            self.animate()
        else:
            self.btn_animate.label.set_text('动画演示')
        
    def animate(self):
        """创建动画"""
        if not self.animation_running:
            return
            
        import matplotlib.animation as animation
        
        def animate_frame(frame):
            if not self.animation_running:
                return
                
            # 创建循环动画
            t = frame * 0.1
            self.theta1_1 = 0.5 + 0.5 * np.sin(t)
            self.theta2_1 = 2.0 + 0.5 * np.cos(t)
            self.theta1_2 = 1.0 + 0.5 * np.sin(t + np.pi/2)
            self.theta2_2 = 2.5 + 0.5 * np.cos(t + np.pi/2)
            
            # 更新滑块值
            self.s_theta1_1.set_val(self.theta1_1)
            self.s_theta2_1.set_val(self.theta2_1)
            self.s_theta1_2.set_val(self.theta1_2)
            self.s_theta2_2.set_val(self.theta2_2)
            
            self.update_plot()
            
            # 继续动画
            if self.animation_running:
                self.fig.canvas.draw()
                plt.pause(0.05)
                self.animate()
        
        # 启动动画
        animate_frame(0)
        
    def update_plot(self):
        """更新绘图"""
        # 计算第一个机构的关节坐标
        joints1 = self.linkage1.forward_kinematics(self.theta1_1, self.theta2_1)
        
        # 计算第二个机构的关节坐标 (向右偏移)
        joints2 = self.linkage2.forward_kinematics(self.theta1_2, self.theta2_2)
        joints2 = [(x + 4, y) for x, y in joints2]  # 向右偏移4个单位
        
        # 更新连杆线条
        self.update_linkage_lines(joints1, self.lines1)
        self.update_linkage_lines(joints2, self.lines2)
        
        # 更新关节点
        self.update_joint_points(joints1, self.joints1)
        self.update_joint_points(joints2, self.joints2)
        
        # 添加机构标签
        self.ax.text(-0.5, 2.5, '机构1', fontsize=14, fontweight='bold', color='blue',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
        self.ax.text(3.5, 2.5, '机构2', fontsize=14, fontweight='bold', color='red',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral", alpha=0.7))
        
        # 显示角度信息
        angle_info = f'机构1: θ1={np.degrees(self.theta1_1):.1f}°, θ2={np.degrees(self.theta2_1):.1f}°\n'
        angle_info += f'机构2: θ1={np.degrees(self.theta1_2):.1f}°, θ2={np.degrees(self.theta2_2):.1f}°'
        self.ax.text(-0.5, -1.5, angle_info, fontsize=11, 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8))
        
        # 显示连杆长度信息
        length_info = f'连杆长度: l1={self.linkage1.l1}, l2={self.linkage1.l2}'
        self.ax.text(-0.5, -1.8, length_info, fontsize=10, 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.8))
        
        self.fig.canvas.draw_idle()
        
    def update_linkage_lines(self, joints, lines):
        """更新连杆线条"""
        # 连杆1: A到B (输入连杆1)
        lines[0].set_data([joints[0][0], joints[1][0]], [joints[0][1], joints[1][1]])
        
        # 连杆2: B到C (中间连杆1)
        lines[1].set_data([joints[1][0], joints[2][0]], [joints[1][1], joints[2][1]])
        
        # 连杆3: C到D (中间连杆2)
        lines[2].set_data([joints[2][0], joints[3][0]], [joints[2][1], joints[3][1]])
        
        # 连杆4: D到E (输入连杆2)
        lines[3].set_data([joints[3][0], joints[4][0]], [joints[3][1], joints[4][1]])
        
    def update_joint_points(self, joints, joint_points):
        """更新关节点"""
        joint_labels = ['A', 'B', 'C', 'D', 'E']
        for i, joint in enumerate(joints):
            joint_points[i].set_data([joint[0]], [joint[1]])
            # 添加关节标签
            self.ax.text(joint[0] + 0.1, joint[1] + 0.1, joint_labels[i], 
                        fontsize=10, fontweight='bold')

def main():
    """主函数"""
    print("五连杆机构正运动学仿真程序")
    print("=" * 60)
    print("功能说明:")
    print("1. 使用滑块控制两个五连杆机构的输入角度θ1和θ2")
    print("2. 蓝色线条表示机构1，红色线条表示机构2")
    print("3. 关节A和E为固定关节，关节B和D为输入关节")
    print("4. 关节C通过几何约束自动计算位置")
    print("5. 点击'重置角度'按钮可以恢复初始角度")
    print("6. 点击'动画演示'按钮可以观看自动动画")
    print("7. 程序实时显示各关节的坐标和角度信息")
    print("=" * 60)
    
    # 创建仿真器
    simulator = FiveBarSimulator()
    
    # 显示图形
    plt.show()

if __name__ == "__main__":
    main() 