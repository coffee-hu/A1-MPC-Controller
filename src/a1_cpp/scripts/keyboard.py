#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys, select, termios, tty

# 函数获取键盘输入
def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# 主控制循环
if __name__ == "__main__":
    # 初始化节点
    rospy.init_node("keyboard_control_node")
    
    # 创建发布者，发布到 /keyboard_topic 话题
    pub = rospy.Publisher("/keyboard_command", String, queue_size=10)
    
    # 设置终端属性以捕获键盘输入
    settings = termios.tcgetattr(sys.stdin)
    
    try:
        print("键盘控制程序启动：")
        print("按 'w', 's', 'a', 'd' 控制方向，'i', 'k' 控制上下，'j', 'l' 控制旋转")
        print("按 'x' 退出程序")

        while not rospy.is_shutdown():
            # 获取键盘输入
            key = get_key()

            # 发布按键
            if key:
                pub.publish(key)

            # 如果按下 'x' 键，则退出
            if key == 'x':
                break

    except Exception as e:
        print(e)

    finally:
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
