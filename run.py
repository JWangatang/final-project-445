"""
Actual helper script to execute code.
It takes care of proper error handling (e.g. if you press CTRL+C) and the difference between
running code on the robot vs. in simulation.

Usage:
  python3 run.py --sim lab1 [for simulation]
  python3 run.py lab1 [to run on a robot]
"""


import sys
import argparse
import importlib


''' USER NOTES ON AUTOMATED TESTING
- Change num_iterations to your liking, currently only runs twice
- Set default value to start from
- Set increment to 0 to keep default value constant
- Image results will be in test_images/
- run_times.txt will have each run's info and run time
'''

# Number of iterations
num_iterations = 2

# THETA VALUES
theta_kp = 300
theta_kp_increment = 50

theta_ki = 5
theta_ki_increment = 5

theta_kd = 50
theta_kd_increment = 10

theta_range = [-180, 180]
theta_range_increment = 0

# DIST VALUES
dist_kp = 1000
dist_kp_increment = 50

dist_ki = 0
dist_ki_increment = 5

dist_kd = 50
dist_kd_increment = 0

dist_range = [-200, 200]
dist_range_increment = 0

# TURN TIME
turn_delta_t = 2
turn_delta_t_increment = 0


class TestSettings:
    def __init__(self, run_num, t_values, t_range, d_values, d_range, turn_d):
        self.run_number = run_num
        self.theta_values = t_values
        self.theta_range = t_range
        self.dist_values = d_values
        self.dist_range = d_range
        self.turn_delta = turn_d

    def to_string(self):
        return "Run " + str(self.run_number) \
               + "\nTheta: " + str(self.theta_values)\
               + "\nTheta Range: " + str(self.theta_range)\
               + "\nDistance: " + str(self.dist_values)\
               + "\nDistance Range: " + str(self.dist_range)\
               + "\nTurn Delta: " + str(self.turn_delta) + "\n"


if __name__ == "__main__":
    file = open("run_times.txt", "w")

    parser = argparse.ArgumentParser()
    parser.add_argument("run", help="run specified module")
    parser.add_argument("--sim", help="Run using VREP simulation", action="store_true")
    args = parser.parse_args()
    clientID = None

    for i in range(0, num_iterations):
        if args.sim:
            from pyCreate2.factory import FactorySimulation
            factory = FactorySimulation()
        else:
            from pyCreate2.factory import FactoryCreate
            factory = FactoryCreate()
        try:
            if args.run.endswith(".py"):
                args.run = args.run[:-3]
            mod = importlib.import_module(args.run)
            Run = getattr(mod, "Run")

            theta_values_test = [theta_kp + theta_kp_increment * i,
                                theta_ki + theta_ki_increment * i,
                                theta_kd + theta_kd_increment * i]

            theta_range_test = [theta_range[0] + theta_range_increment * i,
                                theta_range[1] + theta_range_increment * i]

            dist_values_test = [dist_kp + dist_kp_increment * i,
                                dist_ki + dist_ki_increment * i,
                                dist_kd + dist_kd_increment * i]

            dist_range_test = [dist_range[0] + dist_range_increment * i,
                               dist_range[1] + dist_range_increment * i]

            turn_delta_test = turn_delta_t + turn_delta_t_increment * i

            test_settings = TestSettings(i, theta_values_test, theta_range_test,
                                         dist_values_test, dist_range_test,
                                         turn_delta_test)

            print(test_settings.to_string())

            r = Run(factory, test_settings)
            run_time = r.run()
            file.write("Run " + str(i) + " - " + str(run_time) + "s\n")
            file.write(test_settings.to_string() + "\n")

        except KeyboardInterrupt:
            pass
        except:
            factory.close()
            raise

        factory.close()
    file.close()

    # quit
    sys.exit()
