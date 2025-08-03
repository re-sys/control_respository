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

class FourBarLinkage:
    def __init__(self):
        """
        五连杆机构正运动学计算
        link_lengths: [l1, l2] 各连杆长度
        l1: 固定连杆长度 (A到B,D到A)
        l2: 输入连杆1长度 (B到C，C到D)
        
        """
        self.l1, self.l2 = 1.6, 2
        self.last_point = np.zeros(2)
        self.last_By=0
        self.need_reverse = False
        self.last_length = 0
        
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
        c = 0
        
        
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
        
        
        
        # 计算x的两个解
        x1 = (-B_coeff + np.sqrt(discriminant)) / (2*A_coeff)
        x2 = (-B_coeff - np.sqrt(discriminant)) / (2*A_coeff)
        
        # 计算对应的y值
        if abs(b) < 1e-14:
            if By * self.last_By < 0:
                self.need_reverse = not self.need_reverse
            theta = np.arctan2(abs(Bx),abs(By))
            theta = theta if not self.need_reverse else np.pi-theta
            self.last_By = By
            alpha = np.arcsin(np.sin(theta)*16/20)
            length = 1.6*np.cos(theta)+2*np.cos(alpha)
            length = length if By > 0 else -length
            length = length if length * self.last_length >= 0 else -length
            self.last_length = length
            self.last_point = np.array([0,length])
            return (0,length)
        self.last_length = 0
        self.last_By = By
        self.need_reverse = False
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
    """四连杆机构可视化组件"""
    def __init__(self, x_offset=0, color=QColor(255, 255, 255)):
        super().__init__()
        self.four_bar = FourBarLinkage()
        self.joint_positions = [(0, 0), (0, 0), (0, 0), (0, 0)]
        self.x_offset = x_offset
        self.color = color
        self.setMinimumSize(400, 400)
        
    def update_joints(self, theta1, theta2):
        """更新关节位置"""
        self.joint_positions = self.four_bar.forward_kinematics(theta1, theta2)
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        try:
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
            painter.setPen(QPen(self.color, 3))
            
            # 绘制各连杆
            joints = self.joint_positions
            # 绘制A-B连杆
            if len(joints) > 1:
                painter.drawLine(int((joints[0][0] + self.x_offset) * 100), int(joints[0][1] * 100),
                               int((joints[1][0] + self.x_offset) * 100), int(joints[1][1] * 100))
            # 绘制B-C连杆
            if len(joints) > 2:
                painter.drawLine(int((joints[1][0] + self.x_offset) * 100), int(joints[1][1] * 100),
                               int((joints[2][0] + self.x_offset) * 100), int(joints[2][1] * 100))
            # 绘制C-D连杆
            if len(joints) > 3:
                painter.drawLine(int((joints[2][0] + self.x_offset) * 100), int(joints[2][1] * 100),
                               int((joints[3][0] + self.x_offset) * 100), int(joints[3][1] * 100))
            # 绘制D-A连杆
            if len(joints) >= 4:
                painter.drawLine(int((joints[3][0] + self.x_offset) * 100), int(joints[3][1] * 100),
                               int((joints[0][0] + self.x_offset) * 100), int(joints[0][1] * 100))
            
            # 绘制关节
            painter.setBrush(QBrush(self.color))
            for i, (x, y) in enumerate(joints):
                painter.drawEllipse(int((x + self.x_offset) * 100 - 8), int(y * 100 - 8), 16, 16)
                
            # 绘制关节标签
            painter.setPen(QPen(QColor(255, 255, 0)))
            painter.setFont(QFont("Arial", 10))
            labels = ['A', 'B', 'C', 'D']
            for i, ((x, y), label) in enumerate(zip(joints, labels)):
                painter.drawText(int((x + self.x_offset) * 100 + 15), int(y * 100 + 5), label)
        finally:
            painter.end()

class JointDataThread(QThread):
    """ROS数据接收线程"""
    data_received = pyqtSignal(list)  # 角度值信号
    radians_received = pyqtSignal(list)  # 弧度值信号
    
    def __init__(self):
        super().__init__()
        self.joint_data = [0.0, 0.0, 0.0, 0.0]
        
    def run(self):
        try:
            rclpy.init()
            self.node = Node('joint_visualizer_node')
            self.subscription = self.node.create_subscription(
                JointState,
                '/action',
                self.joint_callback,
                10
            )
            
            while rclpy.ok():
                try:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                except Exception as e:
                    if "ExternalShutdownException" in str(e):
                        break
                    else:
                        print(f"ROS错误: {e}")
                        break
        except Exception as e:
            print(f"初始化错误: {e}")
        finally:
            try:
                if hasattr(self, 'node'):
                    self.node.destroy_node()
                rclpy.shutdown()
            except:
                pass
            
    def joint_callback(self, msg):
        try:
            if len(msg.position) >= 8:
                # 获取前四个关节角度数据用于显示（本来就是弧度值）
                self.joint_data = list(msg.position[:8])
                theta1 = -self.joint_data[0]  # 左前大腿内关节
                theta2 = self.joint_data[1] - math.pi  # 左前大腿外关节
                
                theta3 = -self.joint_data[5]-math.pi  # 左后大腿内关节
                theta4 = self.joint_data[6]  # 左后大腿外关节

                theta_list = [theta1,theta2,theta3,theta4]
                # 转换为角度用于显示
                self.joint_data_degrees = [math.degrees(angle) for angle in theta_list]
                # 发送角度值用于显示
                self.data_received.emit(self.joint_data_degrees)
                # 发送完整的8个弧度值用于四连杆计算
                self.radians_received.emit(self.joint_data)
        except Exception as e:
            print(f"数据处理错误: {e}")

class JointVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.initDataThread()
        
    def initUI(self):
        self.setWindowTitle('四足机器人关节角度可视化')
        self.setGeometry(100, 100, 1400, 800)
        
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
        
        # 顶部角度显示
        angle_display_layout = QHBoxLayout()
        
        # 创建角度显示标签
        self.angle_labels = []
        angle_names = ['左前内', '左前外', '右前内', '右前外']
        
        for i, name in enumerate(angle_names):
            angle_container = QVBoxLayout()
            
            # 角度名称
            name_label = QLabel(name)
            name_label.setAlignment(Qt.AlignCenter)
            name_label.setStyleSheet("color: #00ffff; font-size: 12px; font-weight: bold;")
            angle_container.addWidget(name_label)
            
            # 角度数值
            value_label = QLabel('0.00°')
            value_label.setObjectName("value")
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setStyleSheet("font-size: 14px;")
            angle_container.addWidget(value_label)
            
            self.angle_labels.append(value_label)
            angle_display_layout.addLayout(angle_container)
        
        main_layout.addLayout(angle_display_layout)
        
        # 创建水平布局来放置两个四连杆可视化
        horizontal_layout = QHBoxLayout()
        
        # 左侧：左腿四连杆可视化
        left_layout = QVBoxLayout()
        
        # 左腿标题
        left_title = QLabel('左腿四连杆机构')
        left_title.setObjectName("title")
        left_title.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(left_title)
        
        # 左腿可视化组件
        self.left_linkage_visualizer = LinkageVisualizer(x_offset=-1.5, color=QColor(0, 255, 0))
        left_layout.addWidget(self.left_linkage_visualizer)
        
        # 右侧：右腿四连杆可视化
        right_layout = QVBoxLayout()
        
        # 右腿标题
        right_title = QLabel('右腿四连杆机构')
        right_title.setObjectName("title")
        right_title.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(right_title)
        
        # 右腿可视化组件
        self.right_linkage_visualizer = LinkageVisualizer(x_offset=1.5, color=QColor(0, 0, 255))
        right_layout.addWidget(self.right_linkage_visualizer)
        
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
        self.data_thread.radians_received.connect(self.updateLinkageData)
        self.data_thread.start()
        
    def updateJointData(self, joint_data):
        """更新关节数据显示（角度值）"""
        # 更新顶部角度显示
        for i, (label, value) in enumerate(zip(self.angle_labels, joint_data)):
            label.setText(f'{value:.2f}°')
            
        # 更新状态
        self.status_label.setText('状态: 数据接收正常')
        
    def updateLinkageData(self, joint_data_radians):
        """更新四连杆可视化（弧度值）"""
        # 根据state_machine_controller中的逆变换逻辑
        # joint_cmd[0] = -FL_theta1 (左前大腿内关节)
        # joint_cmd[1] = FL_theta4 + π (左前大腿外关节)
        # joint_cmd[2] = FR_theta1 (右前大腿内关节)
        # joint_cmd[3] = -FR_theta4 - π (右前大腿外关节)
        # joint_cmd[5] = -RL_theta4 - π (左后大腿内关节)
        # joint_cmd[6] = RL_theta1 (左后大腿外关节)
        # joint_cmd[7] = RR_theta4 + π (右后大腿内关节)
        # joint_cmd[8] = -RR_theta1 (右后大腿外关节)
        
        if len(joint_data_radians) >= 8:
            # 左腿：需要反向变换
            theta1 = -joint_data_radians[0]  # 左前大腿内关节
            theta2 = joint_data_radians[1] - math.pi  # 左前大腿外关节
            
            # 右腿：需要反向变换
            # theta3 = joint_data_radians[2]  # 右前大腿内关节
            # theta4 = -joint_data_radians[3] - math.pi  # 右前大腿外关节
# 右腿：需要反向变换
            theta3 = -joint_data_radians[5]-math.pi  # 右前大腿内关节
            theta4 = joint_data_radians[6]  # 右前大腿外关节
            
            # 更新左腿四连杆可视化
            self.left_linkage_visualizer.update_joints(theta1, theta2)
            
            # 更新右腿四连杆可视化
            self.right_linkage_visualizer.update_joints(theta3, theta4)
        
    def closeEvent(self, event):
        """关闭事件处理"""
        try:
            if hasattr(self, 'data_thread'):
                self.data_thread.terminate()
                self.data_thread.wait(timeout=1000)  # 等待1秒
        except Exception as e:
            print(f"关闭线程错误: {e}")
        
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"ROS关闭错误: {e}")
        
        event.accept()

def main():
    try:
        app = QApplication(sys.argv)
        visualizer = JointVisualizer()
        visualizer.show()
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序错误: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main() 