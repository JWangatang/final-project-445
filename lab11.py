"""
Code to draw a given file
Use "python3 run.py --sim lab11" to execute
"""
from pyCreate2 import create2

import lab11_image
import odometry
import pid_controller_soln
import math

class Waypoint:
    def __init__(self, x, y, color, use_pen):
        self.x = x
        self.y = y
        self.color = color
        self.use_pen = use_pen

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
        self.tracker = factory.create_tracker(1, sd_x=0.01, sd_y=0.01, sd_theta=0.01, rate=10)

    def run(self):

        color_dict = { "black":(0.0, 0.0, 0.0), "red":(1.0,0.0,0.0), "green":(0.0,1.0,0.0), "blue":(0.0,0.0,1.0) }

        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        self.penholder.set_color(0.0, 0.0, 0.0)
        
        lines = []

        print("Found %d paths" % len(self.img.paths))
        for path in self.img.paths:
            print("Found %d beziers" % len(path.beziers))
            color = color_dict[path.color]
            lines.append(Waypoint(path.get_start()[0],path.get_start()[1], color, False))
            for i in range(len(path.beziers)):
                for t in range(1,10):
                    curr_point = path.eval(i,t*0.1)
                    # print(path.eval(i,t*0.1))
                    lines.append(Waypoint(curr_point[0],curr_point[1],color,True))
            # lines.append(Waypoint(path.get_start()[0],path.get_start()[1], color, False))
            # lines.append(Waypoint(path.get_end()[0], path.get_end()[1], color, True))

        alpha = .7

        for line in self.img.lines:
            color = color_dict[line.color]

            lines.append(Waypoint(line.u[0], line.u[1], color, False))
            lines.append(Waypoint(line.v[0], line.v[1], color, True))

        for point in lines:
            turn_time = self.time.time() + 2
            color = point.color

            self.penholder.set_color(color[0], color[1], color[2])
            self.raise_pen()
            print("go to:", point.x, point.y, "with color", color)
            while True:
                state = self.create.update()
                r = self.tracker.query()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    x = self.odometry.x
                    y = self.odometry.y
                    theta = self.odometry.theta
                    # if r is not None:
                    #     x = alpha * self.odometry.x + (1 - alpha) * r["position"]["x"]
                    #     y = alpha * self.odometry.y + (1 - alpha) * r["position"]["y"]
                    #     theta = alpha * self.odometry.theta  + (1 - alpha) * r["orientation"]["y"]
                    goal_theta = math.atan2(point.y - y, point.x - x)
                    # print("goal_theta = %f degrees" % math.degrees(goal_theta))
                    # theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                    output_theta = self.pidTheta.update(theta, goal_theta, self.time.time())
                    # if output_theta > 180:
                    #     output_theta = 180
                    # elif output_theta < -180:
                    #     output_theta = -180

                    distance = math.sqrt(math.pow(point.x - x, 2) + math.pow(point.y - y, 2))
                    if distance < 0.02:
                        # print(self.time.time())
                        print("[{},{},{}]".format(x, y, math.degrees(theta)))
                        break

                    if (turn_time > self.time.time()):
                        self.create.drive_direct(int(output_theta), int(-output_theta));
                        self.time.sleep(.001);
                        continue;
                    else:
                        if point.use_pen:
                            self.lower_pen()
                        else:
                            self.raise_pen()

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
