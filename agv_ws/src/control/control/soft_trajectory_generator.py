import numpy as np

class SoftTrajectoryGenerator:
    """
    This class generates a smooth trajectory between waypoints using quintic polynomials.
    It supports adding waypoints with associated times and generating trajectory segments 
    that interpolate the robot's position and orientation (x, y, theta) between them.
    
    Attributes:
        waypoints (list): A list of waypoints, where each waypoint is a dictionary containing 
                           'position' (x, y, theta) and 'time' (time at which the waypoint should be reached).
        trajectorySegments (list): A list of trajectory segments, each containing the polynomial coefficients 
                                   for position and orientation (x, y, theta) over time.
        currentTime (float): The current time in the trajectory generation process.
    """

    def __init__(self):
        """
        Initializes the SoftTrajectoryGenerator by setting up empty lists for waypoints and trajectory segments 
        and initializing the current time to 0.0.
        """
        self.waypoints = []  # List to store the waypoints
        self.trajectorySegments = []  # List to store the trajectory segments
        self.currentTime = 0.0  # Current time in the trajectory generation process

    def addWaypoint(self, position, time):
        """
        Adds a waypoint with a specified position and time.

        Args:
            position (list or np.array): A list or array of length 3, representing [x, y, theta] position.
            time (float): The time at which this waypoint should be reached.
        """
        self.waypoints.append({"position": np.array(position), "time": time})

    def generateTrajectories(self):
        """
        Generates the trajectory segments using quintic polynomials to interpolate between waypoints.
        Each segment consists of polynomial coefficients that define the trajectory for x, y, and theta 
        between consecutive waypoints.
        """
        self.trajectorySegments = []  # Clear the previous trajectory segments

        # Iterate through the waypoints and generate a trajectory segment for each pair of consecutive waypoints
        for i in range(len(self.waypoints) - 1):
            wp0 = self.waypoints[i]
            wp1 = self.waypoints[i + 1]

            # Compute the quintic polynomial coefficients for x, y, and theta
            coeffs_x = self.computeQuinticCoefficients(wp0["time"], wp1["time"], wp0["position"][0], wp1["position"][0], 0, 0, 0, 0)
            coeffs_y = self.computeQuinticCoefficients(wp0["time"], wp1["time"], wp0["position"][1], wp1["position"][1], 0, 0, 0, 0)
            coeffs_theta = self.computeQuinticCoefficients(wp0["time"], wp1["time"], wp0["position"][2], wp1["position"][2], 0, 0, 0, 0)

            # Create the trajectory segment for the current pair of waypoints
            segment = {
                "coeffs_x": coeffs_x,
                "coeffs_y": coeffs_y,
                "coeffs_theta": coeffs_theta,
                "startTime": wp0["time"],
                "endTime": wp1["time"]
            }
            self.trajectorySegments.append(segment)

    def computeQuinticCoefficients(self, t0, tf, p0, pf, v0, vf, a0, af):
        """
        Computes the quintic polynomial coefficients to interpolate between two points (p0, pf)
        with specified velocities (v0, vf) and accelerations (a0, af) at the start (t0) and end (tf) times.

        The general form of the polynomial is:
            p(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5

        Args:
            t0 (float): The start time.
            tf (float): The end time.
            p0 (float): The starting position.
            pf (float): The ending position.
            v0 (float): The starting velocity.
            vf (float): The ending velocity.
            a0 (float): The starting acceleration.
            af (float): The ending acceleration.

        Returns:
            np.array: The array of coefficients [a0, a1, a2, a3, a4, a5].
        """
        # Create the system of equations to solve for the polynomial coefficients
        A = np.array([
            [1, t0, t0**2, t0**3, t0**4, t0**5],
            [1, tf, tf**2, tf**3, tf**4, tf**5],
            [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
            [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
            [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
            [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
        ])
        b = np.array([p0, pf, v0, vf, a0, af])
        
        # Solve the system of equations to get the coefficients
        return np.linalg.solve(A, b)

    def evaluatePolynomial(self, coeffs, t):
        """
        Evaluates the polynomial at a given time 't' using the provided coefficients.

        Args:
            coeffs (np.array): The coefficients of the polynomial.
            t (float): The time at which to evaluate the polynomial.

        Returns:
            float: The value of the polynomial at time 't'.
        """
        return np.polyval(coeffs[::-1], t)

    def getNextState(self, dt):
        """
        Calculates the next position and velocity based on the current time and trajectory segments.

        This function updates the current time and returns the position (x, y, theta) and velocity 
        (vx, vy, vtheta) at that time, by evaluating the corresponding polynomial segment for each 
        waypoint trajectory. If the trajectory has ended, the last known position is held.

        Args:
            dt (float): The time step to advance the trajectory.

        Returns:
            tuple: A tuple containing the position [x, y, theta] and velocity [vx, vy, vtheta].
        """
        self.currentTime += dt  # Increment the current time by the time step
        position = np.zeros(3)  # Position: x, y, theta
        velocity = np.zeros(3)  # Velocity: vx, vy, vtheta

        # Iterate through the trajectory segments and find the corresponding segment for the current time
        for segment in self.trajectorySegments:
            if segment["startTime"] <= self.currentTime <= segment["endTime"]:
                t = self.currentTime
                # Evaluate the position and velocity for x, y, and theta
                position[0] = self.evaluatePolynomial(segment["coeffs_x"], t)
                position[1] = self.evaluatePolynomial(segment["coeffs_y"], t)
                position[2] = self.evaluatePolynomial(segment["coeffs_theta"], t)

                velocity[0] = self.evaluatePolynomial(np.polyder(segment["coeffs_x"]), t)
                velocity[1] = self.evaluatePolynomial(np.polyder(segment["coeffs_y"]), t)
                velocity[2] = self.evaluatePolynomial(np.polyder(segment["coeffs_theta"]), t)
                return position, velocity

        # If the trajectory has ended, hold the last known position
        if self.trajectorySegments:
            last = self.trajectorySegments[-1]
            t = last["endTime"]
            position[0] = self.evaluatePolynomial(last["coeffs_x"], t)
            position[1] = self.evaluatePolynomial(last["coeffs_y"], t)
            position[2] = self.evaluatePolynomial(last["coeffs_theta"], t)

        return position, velocity
