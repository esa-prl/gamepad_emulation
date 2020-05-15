import rclpy
from rclpy.node import Node
from pynput.keyboard import Key, Listener
from sensor_msgs.msg import Joy
from rclpy.callback_groups import ReentrantCallbackGroup

startup_msg = """
This node takes keypresses from the keyboard and publishes them
as Joy messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

Buttons for locomotion modes:
1: Simple rover locomotion
2: Stop mode
3: Wheel walking

ESC to exit
"""
moveBindings = {
    'q': (0.5, 0.5, 0.0, 0.0, 0.0, 0.0),
    'w': (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
    'e': (-0.5, 0.5, 0.0, 0.0, 0.0, 0.0),
    'a': (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    's': (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    'd': (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    'z': (0.5, -0.5, 0.0, 0.0, 0.0, 0.0),
    'x': (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),
    'c': (-0.5, -0.5, 0.0, 0.0, 0.0, 0.0),
}

buttonBindings = {
    '1': (1, 0, 0, 0, 0, 0, 0, 0, 0),
    '2': (0, 0, 1, 0, 0, 0, 0, 0, 0),
    '3': (0, 0, 0, 1, 0, 0, 0, 0, 0),
}


class GamepadEmulation(Node):
    def __init__(self):
        self.node_name = 'gamepad_emulation_node'
        super().__init__(self.node_name)

        print(startup_msg)
        self.pub = self.create_publisher(Joy, 'joy', 1)
        self.joy_msg = Joy()
        self.set_joy_msg(None)

        self.pressed_key = None
        timer_period = 0.5  # seconds
        self.cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(timer_period,
                                       self.timer_callback,
                                       callback_group=self.cb_group)

        listener = Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

    def timer_callback(self):
        self.pub.publish(self.joy_msg)

    def on_press(self, key):
        if (self.pressed_key != key):
            self.pressed_key = key
            self.set_joy_msg(key)
        else:
            pass

    def set_joy_msg(self, key):
        if not hasattr(key, 'char'):
            self.joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        elif key.char in moveBindings.keys():
            self.joy_msg.axes = moveBindings[key.char]
        elif key.char in buttonBindings.keys():
            self.joy_msg.buttons = buttonBindings[key.char]
        else:
            self.joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    def on_release(self, key):
        if key == Key.esc:
            # Stop listener by returning false
            return False
        elif (self.pressed_key == key):
            self.pressed_key = None
            self.set_joy_msg(None)
            # print('Pressed key was released.')

    def stop(self):
        """Shut down method."""
        self.get_logger().info('\t{} STOPPED.'.format(self.node_name.upper()))


def main(args=None):
    """Run node."""
    rclpy.init(args=args)

    # Workaround since there is a bug in stopping python notes properly
    try:
        gamepad_emulation = GamepadEmulation()
        # executor = MultiThreadedExecutor(num_threads=2)
        # executor.add_node(gamepad_emulation)

        try:
            # executor.spin()
            rclpy.spin(gamepad_emulation)
        finally:
            # executor.shutdown()
            gamepad_emulation.stop()
            gamepad_emulation.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
