#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QGridLayout
from PyQt5.QtCore import QTimer, pyqtSignal, QThread, Qt
from PyQt5.QtGui import QFont, QPalette, QColor, QPainter, QPen, QBrush
import numpy as np
import math

class FiveBarLinkage:
    def __init__(self):
        """
        五连杆机构正运动学计算
        link_lengths: [l1, l2, l3, l4, l5] 各连杆长度
        l1: 固定连杆长度 (A到E)
        l2: 输入连杆1长度 (A到B)
        l3: 中间连杆长度 (B到C)
        l4: 输入连杆2长度 (E到D)
        l5: 中间连杆长度 (C到D)
        """
        self.l1, self.l2 = 1.6, 2
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
        
        
        # 计算关节B的坐标 (连杆2的末端)
        B = (self.l1 * np.cos(theta1), self.l1 * np.sin(theta1))
        
        # 计算关节D的坐标 (连杆4的末端)
        D = (self.l1 * np.cos(theta2), self.l1 * np.sin(theta2))
        
        # 计算关节C的坐标 (连杆3和连杆5的交点)
        C = self._solve_joint_c(B, D)
        
        return [A, B, C, D]
    
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
        c = Dx**2 + Dy**2 - Bx**2 - By**2
        
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

class LinkageVisualizer(QWidget):
    """五连杆机构可视化组件"""
    def __init__(self):
        super().__init__()
        self.five_bar = FiveBarLinkage()
        self.joint_positions = [(0, 0), (0, 0), (0, 0), (0, 0)]
        self.setMinimumSize(400, 400)
        
    def update_joints(self, theta1, theta2):
        """更新关节位置"""
        self.joint_positions = self.five_bar.forward_kinematics(theta1, theta2)
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 设置坐标系变换（Y轴向上为正）
        painter.translate(self.width() / 2, self.height() / 2)
        scale = min(self.width(), self.height()) / 400.0  # 自适应缩放
        painter.scale(scale, -scale)
        
        # 绘制背景网格
        painter.setPen(QPen(QColor(50, 50, 50), 1))
        for i in range(-10, 11):
            painter.drawLine(i * 20, -200, i * 20, 200)
            painter.drawLine(-200, i * 20, 200, i * 20)
        
        # 绘制连杆
        painter.setPen(QPen(QColor(255, 255, 255), 3))
        
        # 绘制各连杆
        joints = self.joint_positions
        # 绘制A-B连杆
        if len(joints) > 1:
            painter.drawLine(int(joints[0][0] * 100), int(joints[0][1] * 100),
                           int(joints[1][0] * 100), int(joints[1][1] * 100))
        # 绘制B-C连杆
        if len(joints) > 2:
            painter.drawLine(int(joints[1][0] * 100), int(joints[1][1] * 100),
                           int(joints[2][0] * 100), int(joints[2][1] * 100))
        # 绘制C-D连杆
        if len(joints) > 3:
            painter.drawLine(int(joints[2][0] * 100), int(joints[2][1] * 100),
                           int(joints[3][0] * 100), int(joints[3][1] * 100))
        # 绘制D-E连杆
        if len(joints) >= 4:
            painter.drawLine(int(joints[3][0] * 100), int(joints[3][1] * 100),
                           int(joints[0][0] * 100), int(joints[0][1] * 100))
        
        # 绘制关节
        painter.setBrush(QBrush(QColor(255, 0, 0)))
        for i, (x, y) in enumerate(joints):
            painter.drawEllipse(int(x * 100 - 8), int(y * 100 - 8), 16, 16)
            
        # 绘制关节标签
        painter.setPen(QPen(QColor(255, 255, 0)))
        painter.setFont(QFont("Arial", 12))
        labels = ['A', 'B', 'C', 'D']
        for i, ((x, y), label) in enumerate(zip(joints, labels)):
            painter.drawText(int(x * 100 + 15), int(y * 100 + 5), label)

class JointDataThread(QThread):
    """ROS数据接收线程"""
    data_received = pyqtSignal(list)
    
    def __init__(self):
        super().__init__()
        self.joint_data = [0.0, 0.0, 0.0, 0.0]
        
    def run(self):
        rclpy.init()
        self.node = Node('joint_visualizer_node')
        self.subscription = self.node.create_subscription(
            JointState,
            '/action',
            self.joint_callback,
            10
        )
        
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
    def joint_callback(self, msg):
        if len(msg.position) >= 4:
            # 获取前四个关节角度数据
            self.joint_data = list(msg.position[:4])
            # 转换为角度
            self.joint_data_degrees = [math.degrees(angle) for angle in self.joint_data]
            self.data_received.emit(self.joint_data_degrees)

class JointVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.initDataThread()
        
    def initUI(self):
        self.setWindowTitle('四足机器人关节角度可视化')
        self.setGeometry(100, 100, 1200, 700)
        
        # 设置深色主题
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QLabel {
                background-color: #3c3c3c;
                border: 2px solid #555555;
                border-radius: 10px;
                padding: 10px;
                font-size: 14px;
                font-weight: bold;
            }
            QLabel#title {
                background-color: #4a4a4a;
                border: 2px solid #666666;
                font-size: 18px;
                font-weight: bold;
                color: #00ff00;
            }
            QLabel#value {
                background-color: #1e1e1e;
                border: 2px solid #444444;
                color: #00ffff;
                font-size: 16px;
                font-weight: bold;
            }
        """)
        
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QVBoxLayout(central_widget)
        
        # 标题
        title_label = QLabel('四足机器人关节角度实时监控')
        title_label.setObjectName("title")
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # 创建水平布局来放置关节信息和五连杆可视化
        horizontal_layout = QHBoxLayout()
        
        # 左侧：关节信息
        left_layout = QVBoxLayout()
        
        # 创建网格布局用于显示关节信息
        grid_layout = QGridLayout()
        
        # 关节名称
        joint_names = ['左前大腿内关节', '左前大腿外关节', '右前大腿内关节', '右前大腿外关节']
        joint_units = ['度', '度', '度', '度']
        
        # 创建关节显示标签
        self.joint_labels = []
        for i, (name, unit) in enumerate(zip(joint_names, joint_units)):
            # 关节名称标签
            name_label = QLabel(f'{name}:')
            name_label.setAlignment(Qt.AlignCenter)
            grid_layout.addWidget(name_label, i, 0)
            
            # 关节数值标签
            value_label = QLabel('0.00°')
            value_label.setObjectName("value")
            value_label.setAlignment(Qt.AlignCenter)
            grid_layout.addWidget(value_label, i, 1)
            
            self.joint_labels.append(value_label)
        
        left_layout.addLayout(grid_layout)
        
        # 右侧：五连杆可视化
        right_layout = QVBoxLayout()
        
        # 五连杆标题
        linkage_title = QLabel('五连杆机构可视化')
        linkage_title.setObjectName("title")
        linkage_title.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(linkage_title)
        
        # 五连杆可视化组件
        self.linkage_visualizer = LinkageVisualizer()
        right_layout.addWidget(self.linkage_visualizer)
        
        # 将左右布局添加到水平布局
        horizontal_layout.addLayout(left_layout)
        horizontal_layout.addLayout(right_layout)
        
        main_layout.addLayout(horizontal_layout)
        
        # 添加一些间距
        main_layout.addStretch()
        
        # 状态信息
        status_label = QLabel('状态: 等待数据...')
        status_label.setObjectName("title")
        status_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(status_label)
        self.status_label = status_label
        
    def initDataThread(self):
        """初始化数据接收线程"""
        self.data_thread = JointDataThread()
        self.data_thread.data_received.connect(self.updateJointData)
        self.data_thread.start()
        
    def updateJointData(self, joint_data):
        """更新关节数据显示"""
        for i, (label, value) in enumerate(zip(self.joint_labels, joint_data)):
            label.setText(f'{value:.2f}°')
            
        # 更新五连杆可视化（使用前两个关节角度）
        if len(joint_data) >= 2:
            theta1 = math.radians(joint_data[0])  # 左前大腿内关节
            theta2 = math.radians(joint_data[2])  # 右前大腿内关节
            self.linkage_visualizer.update_joints(theta1, theta2)
            
        # 更新状态
        self.status_label.setText('状态: 数据接收正常')
        
    def closeEvent(self, event):
        """关闭事件处理"""
        if hasattr(self, 'data_thread'):
            self.data_thread.terminate()
            self.data_thread.wait()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    visualizer = JointVisualizer()
    visualizer.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 