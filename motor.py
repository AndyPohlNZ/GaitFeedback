from explorerhat import motor
from threading import Thread
class Motor(Thread):
    def __init__(self, speed):
        super(Motor, self).__init__()
        self.daemon = True
        self.off = False
        self.speed = max(min(float(speed), 50), -50) # sets speed capped at 50 units.

    def run(self):
        """Thread runs continuously unitl stop"""
        while not self.off:
            motor.one.speed(self.speed)
            motor.two.speed(self.speed)

    def stop(self):
        """End this timer thread"""
        self.off = True
        motor.one.speed(0)
        motor.two.speed(0)



if __name__ == "__main__" :
    from time import sleep

    m = Motor(20)
    m.start()
    sleep(5)
    print("stopping motor")
    m.cancel()