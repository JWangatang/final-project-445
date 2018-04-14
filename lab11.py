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
        self.pidDistance = pid_controller_soln.PIDController(900, 0, 50, [0, 0], [-200, 200], is_angle=False)

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

        self.parse_obstacle()

        start1 = Waypoint(.85, 0, None, False)
        end1 = Waypoint(.85, 1, None, False)
        start2 = Waypoint(.45, .1, None, False)
        end2 = Waypoint(.85, .1, None, False)

        print("Intersect Check:", self.intersects_with_obstacle(start1, end1), self.intersects_with_obstacle(start2, end2))

        # Alpha value for Complementary Filtering
        alpha = .3

        # Keep track globally and take into account camera reading when we get one
        x = self.odometry.x
        y = self.odometry.y
        theta = self.odometry.theta

        # Points for the robot to travel to
        waypoints = []

        # Process image, add points
        for path in self.img.paths:
            color = color_dict[path.color]
            waypoints.append(Waypoint(path.get_start()[0], path.get_start()[1], color, False))
            # Bezier curves
            for i in range(len(path.beziers)):
                for t in range(1, 10):
                    curr_point = path.eval(i, t*0.1)
                    # print(path.eval(i,t*0.1))
                    waypoints.append(Waypoint(curr_point[0], curr_point[1], color, True))

        # Decompose lines into waypoints
        for line in self.img.lines:
            color = color_dict[line.color]
            waypoints.append(Waypoint(line.u[0], line.u[1], color, False))
            waypoints.append(Waypoint(line.v[0], line.v[1], color, True))

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
                    if turn_time > self.time.time() and abs(goal_theta - theta) > math.pi / 24:
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

    def parse_obstacle(self):

        # Info about the obstacle in meters
        self.obstacle_center = (.85, .6)
        self.obstacle_radius = .2

    def intersects_with_obstacle(self, start_point, end_point):

        # Create vector between intended destination and the starting position
        endpoint_vec = (end_point.x - start_point.x, end_point.y - start_point.y)

        # Create vector between obstacle center and the starting position
        obstacle_vec = (self.obstacle_center[0] - start_point.x, self.obstacle_center[1] - start_point.y)

        # Find the vector projection of the obstacle_vec on the endpoint_vec
        scalar = (obstacle_vec[0] * endpoint_vec[0] + obstacle_vec[1] * endpoint_vec[1]) / (endpoint_vec[0] ** 2 + endpoint_vec[1] ** 2)
        proj = tuple(scalar * x for x in endpoint_vec)

        # Find the endpoint on the projection from the starting point
        point = tuple([proj[0] + start_point.x, proj[1] + start_point.y])

        # Calculate the distance between the projection endpoint and the center of the obstacle
        dist = math.sqrt((point[1] - self.obstacle_center[1]) ** 2 + (point[0] - self.obstacle_center[0]) ** 2)

        # Calculate the magnitude of the vector projection
        endpoint_vec_mag = math.sqrt(endpoint_vec[0] ** 2 + endpoint_vec[1] ** 2)

        # If the distance from the endpoint of the projection to the center of the obstacle is less than the radius and
        # the magnitude of the projection is at least as big as the radius, then the line from the start point to the end
        # point collides with the obstacle
        if dist <= self.obstacle_radius and endpoint_vec_mag >= self.obstacle_radius:
            return True
        else:
            return False
