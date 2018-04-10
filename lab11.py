"""
Code to draw a given file
Use "python3 run.py --sim lab11" to execute
"""
from pyCreate2 import create2

import lab11_image
import odometry
import pid_controller_soln
import math

class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.penholder = factory.create_pen_holder()
        self.img = lab11_image.VectorImage("lab11_img1.yaml")
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller_soln.PIDController(300, 5, 50, [-10, 10], [-180, 180], is_angle=True)
        self.pidDistance = pid_controller_soln.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        self.penholder.set_color(0.0, 0.0, 0.0)
        
        black_lines = []

        for line in self.img.lines:
            if line.color == "black":
                black_lines.append((line.u[0], line.u[1], False));
                black_lines.append((line.v[0], line.v[1], True));
                print(line.u, line.v, line.color)

        for point in black_lines:
            turn_time = self.time.time() + 2

            self.penholder.go_to(0.0)
            print("go to:", point)
            while True:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(point[1] - self.odometry.y, point[0] - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                    if output_theta > 180:
                        output_theta = 180
                    elif output_theta < -180:
                        output_theta = -180

                    distance = math.sqrt(math.pow(point[0] - self.odometry.x, 2) + math.pow(point[1] - self.odometry.y, 2))
                    if distance < 0.02:
                        # print(self.time.time())
                        print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                        break

                    if (line != black_lines[0] and turn_time > self.time.time()):
                        self.create.drive_direct(int(output_theta), int(-output_theta));
                        self.time.sleep(.01);
                        continue;
                    else:
                        if point[2]:
                            self.penholder.go_to(-0.025)
                        else:
                            self.penholder.go_to(0.0)

                    output_distance = self.pidDistance.update(0, distance, self.time.time())

                    self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
                    self.time.sleep(.01)

        self.create.stop()

    def raise_pen(self):
        self.penholder.go_to(0.0)

    def lower_pen(self):
        self.penholder.go_to(-0.025)

    # Returns True if obstacle is in the robot's path
    def path_is_valid(self, start_point, end_point):
        robot_width_mm = 348.5
        pass

    def closest_point(self):
        pass
