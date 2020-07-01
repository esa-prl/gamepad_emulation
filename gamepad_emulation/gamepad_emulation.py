import rclpy
from rclpy.node import Node
from pynput.keyboard import Key, Listener, KeyCode
from sensor_msgs.msg import Joy
from rclpy.callback_groups import ReentrantCallbackGroup

startup_msg = """
This node takes keypresses from the keyboard and publishes them
as Joy messages. It works best with a US keyboard layout.
---------------------------
Left joystick:
    q    w    e
    a         d
    z    x    c

Right joystick:
    t    y    u
    g         j
    b    n    m

D-Pad:
    arrow keys

Buttons:
    1: X
    2: A
    3: B
    4: Y
    5: LB
    6: RB
    7: LT
    8: RT
    9: Back
    0: Start
    s: Button stick left
    h: Button stick right

ESC to exit
"""

leftJoystickBindings = {
    'q': (0.5, 0.5, 0.0, 0.0, 0.0, 0.0),
    'w': (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
    'e': (-0.5, 0.5, 0.0, 0.0, 0.0, 0.0),
    'a': (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    'd': (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    'z': (0.5, -0.5, 0.0, 0.0, 0.0, 0.0),
    'x': (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),
    'c': (-0.5, -0.5, 0.0, 0.0, 0.0, 0.0),
}

rightJoystickBindings = {
    't': (0.0, 0.0,  0.5,  0.5, 0.0, 0.0),
    'y': (0.0, 0.0,  0.0,  1.0, 0.0, 0.0),
    'u': (0.0, 0.0, -0.5,  0.5, 0.0, 0.0),
    'g': (0.0, 0.0,  1.0,  0.0, 0.0, 0.0),
    'j': (0.0, 0.0, -1.0,  0.0, 0.0, 0.0),
    'b': (0.0, 0.0,  0.5, -0.5, 0.0, 0.0),
    'n': (0.0, 0.0,  0.0, -1.0, 0.0, 0.0),
    'm': (0.0, 0.0, -0.5, -0.5, 0.0, 0.0),
}

dPadBindings = {
    Key.left:  (0.0, 0.0, 0.0, 0.0,  1.0,  0.0),
    Key.right: (0.0, 0.0, 0.0, 0.0, -1.0,  0.0),
    Key.down:  (0.0, 0.0, 0.0, 0.0,  0.0, -1.0),
    Key.up:    (0.0, 0.0, 0.0, 0.0,  0.0,  1.0),
}

buttonBindings = {
    '1': (1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),  # X
    '2': (0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),  # A
    '3': (0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),  # B
    '4': (0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0),  # Y
    '5': (0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),  # LB
    '6': (0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0),  # RB
    '7': (0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0),  # LT
    '8': (0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0),  # RT
    '9': (0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0),  # Back
    '0': (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0),  # Start
    's': (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0),  # Button stick left
    'h': (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),  # Button stick right
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
        if hasattr(key, 'char'):
            key = key.char

        if key in dPadBindings.keys():
            self.joy_msg.axes = dPadBindings[key]
        elif key in leftJoystickBindings.keys():
            self.joy_msg.axes = leftJoystickBindings[key]
        elif key in rightJoystickBindings.keys():
            self.joy_msg.axes = rightJoystickBindings[key]
        elif key in buttonBindings.keys():
            self.joy_msg.buttons = buttonBindings[key]
        else:
            self.joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def on_release(self, key):
        if key == Key.esc:
            # Stop listener by returning false
            return False
        elif (self.pressed_key == key):
            self.pressed_key = None
            self.set_joy_msg(None)

    def stop(self):
        """Shut down method."""
        self.get_logger().info('\t{} STOPPED.'.format(self.node_name.upper()))


def main(args=None):
    """Run node."""
    rclpy.init(args=args)

    # Workaround since there is a bug in stopping python notes properly
    try:
        gamepad_emulation = GamepadEmulation()

        try:
            rclpy.spin(gamepad_emulation)
        finally:
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
