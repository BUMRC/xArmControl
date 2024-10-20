import time
import easyhid
import numpy as np

def itos(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb

class XArm:
    def __init__(self, pid=22352):
        # Stores an enumeration of all the connected USB HID devices
        en = easyhid.Enumeration()

        # Return a list of devices based on the search parameters
        devices = en.find(vid=1155, pid=pid)

        # Print a description of the devices found (if needed for debugging)
        # for dev in devices:
        #     print(dev.description())

        assert len(devices) > 0, "No xArm devices found"
        self.dev = devices[0]

        # Open a device
        self.dev.open()
        print('Connected to xArm device')

    def __del__(self):
        print('Closing xArm device')
        self.dev.close()

    def move_to(self, id, pos, time=0):
        """
        CMD_SERVO_MOVE
        0x55 0x55 len 0x03 count [time_lsb time_msb, id, pos_lsb pos_msb]
        Servo position is in range [0, 1000]
        """
        t_lsb, t_msb = itos(time)
        p_lsb, p_msb = itos(pos)
        self.dev.write([0x55, 0x55, 8, 0x03, 1, t_lsb, t_msb, id, p_lsb, p_msb])

    def move_all(self, poss, time=0):
        """
        Set the position of all servos at once
        """
        for i in range(6):
            self.move_to(id=i+1, pos=poss[i], time=time)

    def servos_off(self):
        self.dev.write([0x55, 0x55, 9, 20, 6, 1, 2, 3, 4, 5, 6])

    def read_pos(self):
        """
        Read the position of all 6 servos
        """
        
        self.dev.write([0x55, 0x55, 9, 21, 6, 1, 2, 3, 4, 5, 6])
        ret = self.dev.read()
        
        count = ret[4]
        assert count == 6, "Expected 6 servo positions"

        poss = []
        for i in range(6):
            p_lsb = ret[5 + 3*i + 1]
            p_msb = ret[5 + 3*i + 2]
            pos = (p_msb << 8) + p_lsb
            poss.append(pos)

        return np.array(poss)

    def rest(self):
        self.move_all([500, 500, 200, 900, 800, 500], time=1500)
        time.sleep(2)
        self.servos_off()

class SafeXArm:
    """
    Wrapper to limit motion range and speed to maximize durability
    """
    def __init__(self, **kwargs):
        self.arm = XArm(**kwargs)

        self.min_pos = np.array([100, 200, 400, 100, 50, 200])
        self.max_pos = np.array([900, 800, 900, 600, 850, 650])
        self.max_speed = 250  # Maximum movement speed (range/second)

        self.move_all([0] * 6)
        time.sleep(2)

    def read_pos(self):
        return np.flip(self.arm.read_pos(), 0)

    def rest(self):
        return self.arm.rest()

    def move_all(self, pos):
        pos = np.array(pos) if not isinstance(pos, np.ndarray) else pos

        pos = (pos + 1) / 2
        target = self.min_pos + pos * (self.max_pos - self.min_pos)
        target = np.flip(target, 0).astype(np.uint16)

        for i in range(6):
            self.arm.move_to(id=i+1, pos=target[i], time=100)

def demo():
    arm = SafeXArm()

    # Move to the right
    arm.move_all([-1, 0, 0, 0, 0, 0])
    print(arm.read_pos())
    time.sleep(2)

    # Move to the left
    arm.move_all([1, 0, 0, 0, 0, 0])
    st = time.time()
    print(arm.read_pos())
    print(time.time() - st)
    time.sleep(2)

    # Move to default position
    arm.move_all([0, 0, 0, 0, 0, 0])
    print(arm.read_pos())
    time.sleep(2)

    # Open gripper
    arm.move_all([0, 0, 0, 0, 0, -1])
    print(arm.read_pos())
    time.sleep(2)

    # Close gripper
    arm.move_all([0, 0, 0, 0, 0, 1])
    print(arm.read_pos())
    time.sleep(2)

    # Put the arm back in a resting position
    arm.rest()

demo()
