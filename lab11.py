"""
Code to draw a given file
Use "python3 run.py --sim lab11" to execute
"""
from pyCreate2 import create2

import lab11_image
import odometry
import pid_controller_soln
import math

# Pen Color RGB Values
color_dict = {"black": (0.0, 0.0, 0.0),
              "red": (1.0, 0.0, 0.0),
              "green": (0.0, 1.0, 0.0),
              "blue": (0.0, 0.0, 1.0)}


# Points in Path Traveled
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

        # Sensors
        self.odometry = odometry.Odometry()
        self.tracker = factory.create_tracker(1, sd_x=0.01, sd_y=0.01, sd_theta=0.01, rate=10)

        # Controllers for Odometry
        self.pidTheta = pid_controller_soln.PIDController(300, 5, 50, [-10, 10], [-180, 180], is_angle=True)
        self.pidDistance = pid_controller_soln.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

        # Pen Holder - raises and lowers pen
        self.penholder = factory.create_pen_holder()

        # Image to be drawn
        self.img = lab11_image.VectorImage("lab11_img1.yaml")

    def run(self):

        self.create.start()
        self.create.safe()

        # Request Sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        # Array of Waypoints in the lines to be drawn
        lines = []

        # Process image, add points
        for path in self.img.paths:
            color = color_dict[path.color]
            lines.append(Waypoint(path.get_start()[0], path.get_start()[1], color, False))
            # Bezier curves
            for i in range(len(path.beziers)):
                for t in range(1, 10):
                    curr_point = path.eval(i, t*0.1)
                    # print(path.eval(i,t*0.1))
                    lines.append(Waypoint(curr_point[0], curr_point[1], color, True))

        # Points for the robot to travel to
        waypoints = []

        # Alpha value for Complementary Filtering
        alpha = .7

        # Keep track globally and take into account camera reading when we get one
        x = self.odometry.x
        y = self.odometry.y
        theta = self.odometry.theta

        # Decompose lines into waypoints
        for line in self.img.lines:
            color = color_dict[line.color]
            lines.append(Waypoint(line.u[0], line.u[1], color, False))
            lines.append(Waypoint(line.v[0], line.v[1], color, True))

        #
        for point in waypoints:
            # Time for robot to turn before moving to point
            turn_time = self.time.time() + 2

            # Set color, raise pen until robot reaches point
            color = point.color
            self.penholder.set_color(color[0], color[1], color[2])
            self.raise_pen()

            while True:
                # Take odometry and camera readings
                state = self.create.update()
                r = self.tracker.query()

                # Update the odometry
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    x += self.odometry.delta_d * math.cos(self.odometry.last_theta)
                    y += self.odometry.delta_d * math.sin(self.odometry.last_theta)
                    theta = math.fmod(theta + self.odometry.delta_theta, 2 * math.pi)

                    # Apply Complementary Filter with camera reading
                    if r is not None:
                        x *= alpha
                        y *= alpha
                        theta *= alpha
                        x += (1 - alpha) * r["position"]["x"]
                        y += (1 - alpha) * r["position"]["y"]
                        theta += (1 - alpha) * r["orientation"]["y"]

                    # Calculate the desired angle so the robot faces the goal and apply controller
                    goal_theta = math.atan2(point.y - y, point.x - x)
                    output_theta = self.pidTheta.update(theta, goal_theta, self.time.time())

                    # Check to see if we are close enough to our goal
                    distance = math.sqrt(math.pow(point.x - x, 2) + math.pow(point.y - y, 2))
                    if distance < 0.02:
                        print("[{},{},{}]".format(x, y, math.degrees(theta)))
                        break

                    # Take the first two seconds of every waypoint to orient towards the goal
                    if turn_time > self.time.time():
                        self.create.drive_direct(int(output_theta), int(-output_theta));
                        self.time.sleep(.001);
                        continue;
                    else:
                        if point.use_pen:
                            self.lower_pen()
                        else:
                            self.raise_pen()

                    # Apply controller to distance and drive
                    output_distance = self.pidDistance.update(0, distance, self.time.time())
                    self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
                    self.time.sleep(.01)

        while True:
            continue
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
