"""
Code to draw a given file
Use "python3 run.py --sim lab11" to execute

SIMULATION
CODE

"""
from pyCreate2 import create2

import lab11_image
import odometry
import pid_controller_soln
import math

from PIL import Image, ImageDraw

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
        self.point = (x,y)

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
        self.pidDistance = pid_controller_soln.PIDController(1000, 0, 50, [0, 0], [-500, 500], is_angle=False)

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

        # Alpha value for Complementary Filtering
        alpha = .6

        # Keep track globally and take into account camera reading when we get one
        x = self.odometry.x
        y = self.odometry.y
        theta = self.odometry.theta

        # Points for the robot to travel to
        waypoints = []

        # Decompose lines into waypoints
        # im = Image.new('RGB',(512,512))
        # im_draw = ImageDraw.Draw(im)

        black_lines = []
        green_lines = []
        blue_lines = []
        red_lines = []

        for line in self.img.lines:
            if line.color == "black":
                black_lines.append(line)
            elif line.color == "green":
                green_lines.append(line)
            elif line.color == "blue":
                blue_lines.append(line)
            elif line.color == "red":
                red_lines.append(line)

        for i in range(0, len(black_lines)):
            line = black_lines[i]
            color = color_dict[line.color]
            start_point = Waypoint(line.u[0], line.u[1], color, False)
            end_point = Waypoint(line.v[0], line.v[1], color, True)
            alt_lines = self.alt_lines(start_point, end_point)
            waypoints.append(alt_lines[1-i][0]);
            waypoints.append(alt_lines[1-i][1])

        for i in range(0, len(green_lines)):
            line = green_lines[i]
            color = color_dict[line.color]
            start_point = Waypoint(line.u[0], line.u[1], color, False)
            end_point = Waypoint(line.v[0], line.v[1], color, True)
            alt_lines = self.alt_lines(start_point, end_point)
            waypoints.append(alt_lines[1][0])
            waypoints.append(alt_lines[1][1])

        for i in range(0, len(blue_lines)):
            line = blue_lines[i]
            color = color_dict[line.color]
            start_point = Waypoint(line.u[0], line.u[1], color, False)
            end_point = Waypoint(line.v[0], line.v[1], color, True)
            alt_lines = self.alt_lines(start_point, end_point)
            if i == 0:
                waypoints.append(alt_lines[1][0])
                waypoints.append(alt_lines[1][1])
            else:
                waypoints.append(alt_lines[0][0])
                waypoints.append(alt_lines[0][1])

        red_lines.reverse()

        for i in range(0, len(red_lines)):
            line = red_lines[i]
            color = color_dict[line.color]
            start_point = Waypoint(line.u[0], line.u[1], color, False)
            end_point = Waypoint(line.v[0], line.v[1], color, True)
            alt_lines = self.alt_lines(start_point, end_point)
            waypoints.append(alt_lines[1-i][0])
            waypoints.append(alt_lines[1-i][1])

        # Process image, add points
        for path in self.img.paths:
            color = color_dict[path.color]
            d = 348.5/2/1000
            waypoints.append(Waypoint(path.get_start()[0], path.get_start()[1] + d, color, False))
            # Bezier curves
            for i in range(len(path.beziers)):
                for t in range(1, 5):
                    curr_point = path.eval(i, t*0.25)
                    # print(path.eval(i,t*0.1))
                    waypoints.append(Waypoint(curr_point[0], curr_point[1] + d, color, True))

        # Previous pen color stored to indicate color changes
        prev_pen_color = waypoints[0].color

        turn_delta_t = 1.8
        for point in waypoints:
            # Time for robot to turn before moving to point
            turn_time = self.time.time() + turn_delta_t

            # Set color, raise pen until robot reaches point
            color = point.color
            self.penholder.set_color(color[0], color[1], color[2])
            self.raise_pen()

            # Pauses to change pen. Runs again when 'enter' is pressed
            # Note: can be commented out for simulation
            # if prev_pen_color is not color:
            #     self.change_pen_color(color)
            #     prev_pen_color = color

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
                        # print("Got r: camera_x = %f, camera_y = %f, camera_theta = %f degrees" % (r["position"]["x"],r["position"]["y"],r["orientation"]["y"]))
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
                        self.create.drive_direct(0, 0)
                        break

                    # Take the first two seconds of every waypoint to orient towards the goal
                    if turn_time > self.time.time():
                        self.create.drive_direct(int(output_theta), int(-output_theta))
                        self.time.sleep(.001)
                        continue
                    else:
                        if point.use_pen:
                            self.lower_pen()
                        else:
                            self.raise_pen()

                    # Apply controller to distance and drive
                    output_distance = self.pidDistance.update(0, distance, self.time.time())
                    theta_factor = 0.4
                    if distance < 0.1:
                        theta_factor = 0
                    self.create.drive_direct(int(output_theta * theta_factor + output_distance),
                                             int(-output_theta * theta_factor + output_distance))
                    self.time.sleep(.01)

        while True:
            continue
        self.create.stop()

    def raise_pen(self):
        self.penholder.go_to(0.0)

    def lower_pen(self):
        self.penholder.go_to(-0.025)

    def change_pen_color(self, color):
        for name, rgb in color_dict.items():  # for name, age in list.items():  (for Python 3.x)
            if rgb is color:
                input("Change pen color to: " + str(name))

    # returns two alternate lines, given a line
    # if the robot follows either of the alternate lines, the pen will trace the original line
    def alt_lines(self, start_point, end_point):
        # radius of robot
        d = 348.5/2/1000

        # there are two possible lines (plus_line, min_line) that trace the original line (og_line)
        # plus_line always starts at the "top" of og_line
        # min_line always starts at the "bottom" of og_line
        # if og_line is horizontal, plus_line starts at the left of og_line, min_line at the right
        p0 = None
        p1 = None

        if start_point.x == end_point.x:
            if start_point.y < end_point.y:
                p0 = (start_point.x,start_point.y)
                p1 = (end_point.x,end_point.y)
            else:
                p0 = (end_point.x,end_point.y)
                p1 = (start_point.x,start_point.y)
        elif start_point.x < end_point.x:
            p0 = (start_point.x,start_point.y)
            p1 = (end_point.x,end_point.y)
        else:
            p0 = (end_point.x,end_point.y)
            p1 = (start_point.x,start_point.y)

        m_perp = 0 # default value assumes vertical line
        if (end_point.x == start_point.x and end_point.y == start_point.y):
            return None
        elif (end_point.x == start_point.x):  # check for vertical line
            m_perp = 0
        elif (end_point.y == start_point.y):    # check for horizontal line
            # can't just take -1/m_og because that has div by 0, have to compute by hand
            # perpendicular line is vertical: add or subtract to y, keep same x
            plus_start = (p0[0], p0[1] + d)
            plus_end = (p1[0], p1[1] + d)
            minus_start = (p1[0], p1[1] - d)
            minus_end = (p0[0], p0[1] - d)
            # x1_plus_start = p0[0]
            # x1_plus_end = p1[0]
            # y1_plus_start = p0[1] + d
            # y1_plus_end = p1[1] + d

            plus_start_wp = Waypoint(plus_start[0],plus_start[1],start_point.color, False)
            plus_end_wp = Waypoint(plus_end[0],plus_end[1],start_point.color,True)
            # plus_start_wp = Waypoint(x1_plus_start, y1_plus_start, start_point.color, False)
            # plus_end_wp = Waypoint(x1_plus_end, y1_plus_end, end_point.color, True)

            minus_start_wp = Waypoint(minus_start[0],minus_start[1],start_point.color, False)
            minus_end_wp = Waypoint(minus_end[0],minus_end[1],start_point.color,True)

            # print("Original line: (%f,%f)->(%f,%f)" % (start_point.x, start_point.y, end_point.x, end_point.y))
            # print("Plusle: (%f,%f)->(%f,%f)" % (plus_start_wp.x, plus_start_wp.y, plus_end_wp.x, plus_end_wp.y))
            # print("Minun: (%f,%f)->(%f,%f)" % (minus_start_wp.x, minus_start_wp.y, minus_end_wp.x, minus_end_wp.y))

            return ((plus_start_wp,plus_end_wp),(minus_start_wp,minus_end_wp))

        else:
            m0 = float(end_point.y - start_point.y)/((end_point.x)-(start_point.x))
            m_perp = -1/m0

        x1_plus_start = 0
        x1_plus_end = 0
        y1_plus_start = 0
        y1_plus_end = 0
        x1_minus_start = 0
        x1_minus_end = 0
        y1_minus_start = 0
        y1_minus_end = 0

        # original line sloping up
        if m_perp <= 0:
            x1_plus_start = p1[0] + d * math.sqrt(1/(1+m_perp**2))
            y1_plus_start = p1[1] + m_perp * d * math.sqrt(1/(1+m_perp**2))
            x1_minus_end = p1[0] - d * math.sqrt(1/(1+m_perp**2))
            y1_minus_end = p1[1] - m_perp * d * math.sqrt(1/(1+m_perp**2))
            
            x1_plus_end = p0[0] + d * math.sqrt(1/(1+m_perp**2))
            y1_plus_end = p0[1] + m_perp * d * math.sqrt(1/(1+m_perp**2))
            x1_minus_start = p0[0] - d * math.sqrt(1/(1+m_perp**2))
            y1_minus_start = p0[1] - m_perp * d * math.sqrt(1/(1+m_perp**2))

        else:   # original line sloping down
            x1_plus_start = p0[0] + d * math.sqrt(1/(1+m_perp**2))
            y1_plus_start = p0[1] + m_perp * d * math.sqrt(1/(1+m_perp**2))
            x1_minus_end = p0[0] - d * math.sqrt(1/(1+m_perp**2))
            y1_minus_end = p0[1] - m_perp * d * math.sqrt(1/(1+m_perp**2))

            x1_plus_end = p1[0] + d * math.sqrt(1/(1+m_perp**2))
            y1_plus_end = p1[1] + m_perp * d * math.sqrt(1/(1+m_perp**2))
            x1_minus_start = p1[0] - d * math.sqrt(1/(1+m_perp**2))
            y1_minus_start = p1[1] - m_perp * d * math.sqrt(1/(1+m_perp**2))

        plus_start_wp = Waypoint(x1_plus_start, y1_plus_start, start_point.color, False)
        plus_end_wp = Waypoint(x1_plus_end, y1_plus_end, end_point.color, True)
        minus_start_wp = Waypoint(x1_minus_start, y1_minus_start, start_point.color, False)
        minus_end_wp = Waypoint(x1_minus_end, y1_minus_end, end_point.color, True)

        return ((plus_start_wp,plus_end_wp),(minus_start_wp,minus_end_wp))

        # return (plus_start_wp,plus_end_wp)
