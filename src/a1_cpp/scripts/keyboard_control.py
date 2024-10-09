#! /usr/bin/env python
import rospy,select,termios,tty,sys
from std_msgs.msg import String

## 获取键盘按键信息
def listenKey():
    ## sys.stdin表示标准化输入
    ## termios.tcgetattr(fd)返回一个包含文件描述符fd的tty属性的列表
    property_list = termios.tcgetattr(sys.stdin)
    ## tty.setraw(fd, when=termios.TCSAFLUSH)将文件描述符fd的模式更改为raw。如果when被省略，则默认为termios.TCSAFLUSH，并传递给termios.tcsetattr()
    tty.setraw(sys.stdin.fileno())
    ## 第一个参数是需要监听可读的套接字, 第二个是需要监听可写的套接字, 第三个是需要监听异常的套接字, 第四个是时间限制设置
    ## 如果监听的套接字满足了可读可写条件, 那么所返回的can_read 或 can_write就会有值, 然后就可以利用这些返回值进行后续操作
    can_read, _, _ = select.select([sys.stdin], [], [], 0.1)
    if can_read:
    	keyValue = sys.stdin.read(1)
    else:
    	keyValue = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, property_list)
    return keyValue




if __name__ == "__main__":
    rospy.init_node("keyboard_control")
    print("command: \n \
0: disable; s: switch pos/force control; c: shutdown; \n \
1: stand; 2: lie down;            (during pos control)\n \
a: change gait;                 (during force control)\n \
move_command: \n \
u i o \n \
j k l \n \
m , .")
    pub = rospy.Publisher("/keyboard_command",String,queue_size=10)
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():

        #拼接字符串
        key = listenKey()
        msg = String()
        msg.data = key
        pub.publish(msg)
        rate.sleep()
        if key == 'c':
            rospy.signal_shutdown("shutdown")
        elif key == 'i':
            print("move_forward")
        elif key == 'j':
            print("turn_left")
        elif key == 'l':
            print("turn_right")
        elif key == 'u':
            print("forward_left")
        elif key == 'o':
            print("forward_right")
        elif key == 'k':
            print("stop!")
        elif key == 'a':
            print("change gait")
        elif key == '0':
            print("disable")
        elif key == '1':
            print("stand up")
        elif key == '2':
            print("lie down")

