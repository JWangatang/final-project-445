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
            start_point = Waypoint(line.u[0], line.u[1], color, False)
            end_point = Waypoint(line.v[0], line.v[1], color, True)
            # lines.append(Waypoint(line.u[0], line.u[1], color, False))
            # lines.append(Waypoint(line.v[0], line.v[1], color, True))
            alt_lines = self.alt_lines(start_point, end_point)
            waypoints.append(alt_lines[0][0])
            waypoints.append(alt_lines[0][1])

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

    # returns two alternate lines, given a line
    # if the robot follows either of the alternate lines, the pen will trace the original line
    def alt_lines(self, start_point, end_point):
        # radius of robot
        d = 348.5/2/1000

        # there are two possible lines that trace the original line, hereby referred to as plusle and minun
        # If m0 > 0, then plusle has the same direction as the original line
        # If m0 < 0, then plusle has the opposite direction as the original line
        # If m0 = 0 or infinity, then we have a special case

        x0 = start_point.x
        y0 = start_point.y

        m1 = 0 # default value assumes vertical line
        if (end_point.x == start_point.x and end_point.y == start_point.y):
            return None
        elif (end_point.x == start_point.x):  # check for vertical line
            m1 = 0
        elif (end_point.y == start_point.y):    # check for horizontal line
            # no value of m1 makes sense, have to compute by hand
            # perpendicular line is vertical: add or subtract to x, keep same y
            x1_plus_start = x0 + d
            x1_plus_end = x0 + d
            y1_plus_start = max(start_point.y, end_point.y)
            y1_plus_end = min(start_point.y, end_point.y)

            x1_minus_start = x0 - d
            x1_minus_end = x0 - d
            y1_minus_start = y1_plus_end
            y1_minus_end = y1_plus_start

            plus_start_wp = Waypoint(x1_plus_start, y1_plus_start, start_point.color, False)
            plus_end_wp = Waypoint(x1_plus_end, y1_plus_end, end_point.color, True)

            minus_start_wp = Waypoint(x1_minus_start, y1_minus_start, start_point.color, False)
            minus_end_wp = Waypoint(x1_minus_end, y1_minus_end, end_point.color, True)

            return ((plus_start_wp,plus_end_wp),(minus_start_wp,minus_end_wp))

        else:
            m0 = float(end_point.y - start_point.y)/(end_point.x)-(start_point.x)
            m1 = -1/m0

        x1_plus_start = 0
        x1_plus_end = 0
        y1_plus_start = 0
        y1_plus_end = 0
        x1_minus_start = 0
        x1_minus_end = 0
        y1_minus_start = 0
        y1_minus_end = 0

        if m1 <= 0:
            p0 = None
            if (start_point.x < end_point.x):
                p0 = start_point
            else:
                p0 = end_point

            x1_plus_start = p0.x + d * math.sqrt(1/(1+m1**2))
            y1_plus_start = p0.y + m1 * d * math.sqrt(1/(1+m1**2))
            x1_minus_end = p0.x - d * math.sqrt(1/(1+m1**2))
            y1_minus_end = p0.y - m1 * d * math.sqrt(1/(1+m1**2))

            if (start_point.x < end_point.x):
                p0 = end_point
            else:
                p0 = start_point
            
            x1_plus_end = p0.x + d * math.sqrt(1/(1+m1**2))
            y1_plus_end = p0.y + m1 * d * math.sqrt(1/(1+m1**2))
            x1_minus_start = p0.x - d * math.sqrt(1/(1+m1**2))
            y1_minus_start = p0.y - m1 * d * math.sqrt(1/(1+m1**2))
        else:
            p0 = None
            if (start_point.x < end_point.x):
                p0 = start_point
            else:
                p0 = end_point

            x1_plus_start = p0.x + d * math.sqrt(1/(1+m1**2))
            y1_plus_start = p0.y + m1 * d * math.sqrt(1/(1+m1**2))
            x1_minus_end = p0.x - d * math.sqrt(1/(1+m1**2))
            y1_minus_end = p0.y - m1 * d * math.sqrt(1/(1+m1**2))

            if (start_point.x < end_point.x):
                p0 = end_point
            else:
                p0 = start_point
            
            x1_plus_end = p0.x + d * math.sqrt(1/(1+m1**2))
            y1_plus_end = p0.y + m1 * d * math.sqrt(1/(1+m1**2))
            x1_minus_start = p0.x - d * math.sqrt(1/(1+m1**2))
            y1_minus_start = p0.y - m1 * d * math.sqrt(1/(1+m1**2))

        plus_start_wp = Waypoint(x1_plus_start, y1_plus_start, start_point.color, False)
        plus_end_wp = Waypoint(x1_plus_end, y1_plus_end, end_point.color, True)
        minus_start_wp = Waypoint(x1_minus_start, y1_minus_start, start_point.color, False)
        minus_end_wp = Waypoint(x1_minus_end, y1_minus_end, end_point.color, True)

        print("Original line: (%f,%f)->(%f,%f)" % (start_point.x, start_point.y, end_point.x, end_point.y))
        print("Plusle: (%f,%f)->(%f,%f)" % (plus_start_wp.x, plus_start_wp.y, plus_end_wp.x, plus_end_wp.y))

        return ((plus_start_wp,plus_end_wp),(minus_start_wp,minus_end_wp))
