"""
Code to draw a given file
Use "python3 run.py --sim lab11" to execute
"""
from pyCreate2 import create2

import lab11_image
import odometry
import pid_controller_soln
import math

#from collections import defaultdict

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

    def to_string(waypoint):
        return "x: " + str(waypoint.x) + \
               " y: " + str(waypoint.y) + \
               " color: " + waypoint.color + \
               " use_pen: " + str(waypoint.use_pen)


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

        # Alpha value for Complementary Filtering
        alpha = .7

        # Keep track globally and take into account camera reading when we get one
        x = self.odometry.x
        y = self.odometry.y
        theta = self.odometry.theta

        # Points for the robot to travel to
        waypoints = []

        # Process image, add points
        # for path in self.img.paths:
        #     color = color_dict[path.color]
        #     waypoints.append(Waypoint(path.get_start()[0], path.get_start()[1], color, False))
        #     # Bezier curves
        #     for i in range(len(path.beziers)):
        #         for t in range(1, 10):
        #             curr_point = path.eval(i, t*0.1)
        #             # print(path.eval(i,t*0.1))
        #             waypoints.append(Waypoint(curr_point[0], curr_point[1], color, True))

        # Dictionary grouping lines by color
        # {color : [(start_point, end_point), ... ]}
        line_color_dic = {}
        line_colors_left = []

        # Decompose lines into waypoints
        for line in self.img.lines:
            line_colors_left.append(line.color)
            # Create dictionary of color and list of start/end points of lines
            rgb_value = color_dict[line.color]

            if line.color in line_color_dic:
                line_color_dic[line.color].append((Waypoint(line.u[0], line.u[1], rgb_value, False),
                                              Waypoint(line.v[0], line.v[1], rgb_value, True)));
            else:
                line_color_dic[line.color] = [(Waypoint(line.u[0], line.u[1], rgb_value, False),
                                              Waypoint(line.v[0], line.v[1], rgb_value, True))]

        curr_color = line_colors_left[0]
        line_colors_left.remove(curr_color)

        waypoints.append(line_color_dic[curr_color][0])
        waypoints.append(line_color_dic[curr_color][1])
        curr_point = line_color_dic[curr_color][1]



        while True:

            copied_list = []
            if not line_color_dic[curr_color]:
                #cu
                pass


        for color in line_color_dic:
            # list is not empty
            if not line_color_dic[color]:
                closest_line = None
                shortest_dist = None

                for line in line_color_dic[color]:
                    if closest_line is None:
                        closest_line = line

                        continue
                    else:
                        pass


                    # line is a tuple
                    start_point = line[0]
                    end_point = line[1]

                    if self.distance_between_points(start_point, curr_point) < self.distance_between_points(end_point, curr_point):
                        waypoints.append(start_point)
                        waypoints.append(end_point)
                        curr_point = end_point
                    else:
                        waypoints.append(end_point)
                        waypoints.append(start_point)
                        curr_point = start_point


                pass

            continue



            print(first_start_point.x, first_start_point.y)


            #print(color)
            #print(line_color_dic[color])

        #print(line_color_dic)
        input("...")

        '''
        1) Create dictionary of color and list of start/end points of lines
            - dictionary: [color, [(start point of line, end point of line), ...]
            
        2) waypoints = finished list of points
        
        curr_point = last point added to waypoints
        curr_color = last color added to waypoints
        
        while True
            copied
        
        
            if the list of points for the current color is not empty:
                find the closest point in the list to the current point
                add to waypoints
                remove points **  
                update current point
            
                
        
        
        
        
        
            
        3) Take first line l in dictionary of a color
            a) remove l from dictionary
            b) add start/end of l to waypoints
            
        4/c) While there are lines in dictionary of the same color
            i) find closest point (start or end) to end point of l
            ii) if the closest point is end:
                - reverse start/end points in dictionary
            iii) repeat c
        5/d) take last end point and closest point (start and end) of lines in dictionary
            i) if closest is end, reverse points in dictionary
            ii) Repeat a for all lines of that color
        '''

        # Previous pen color stored to indicate color changes
        prev_pen_color = waypoints[0].color

        for point in waypoints:
            # Time for robot to turn before moving to point
            turn_time = self.time.time() + 2

            # Set color, raise pen until robot reaches point
            color = point.color
            self.penholder.set_color(color[0], color[1], color[2])
            self.raise_pen()

            # Pauses to change pen. Runs again when 'enter' is pressed
            # Note: can be commented out for simulation
            if prev_pen_color is not color:
                self.change_pen_color(color)
                prev_pen_color = color

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

    def change_pen_color(self, color):
        for name, rgb in color_dict.items():  # for name, age in list.items():  (for Python 3.x)
            if rgb is color:
                input("Change pen color to: " + str(name))

    # Returns True if obstacle is in the robot's path
    def path_is_valid(self, start_point, end_point):
        robot_width_mm = 348.5
        pass

    # Distance between 2 Waypoints
    def distance_between_points(self, p1, p2):
        return math.sqrt(math.pow(p2.x - p1.x, 2) + math.pow(p2.y - p2.y, 2))
