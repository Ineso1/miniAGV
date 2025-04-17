import numpy as np

class SoftTrajectoryGenerator:
    def __init__(self):
        self.waypoints = []
        self.trajectorySegments = []
        self.currentTime = 0.0

    def addWaypoint(self, position, time):
        """
        position: [x, y, theta]
        time: time (s) at which this waypoint should be reached
        """
        self.waypoints.append({"position": np.array(position), "time": time})

    def generateTrajectories(self):
        self.trajectorySegments = []

        for i in range(len(self.waypoints) - 1):
            wp0 = self.waypoints[i]
            wp1 = self.waypoints[i + 1]

            coeffs_x = self.computeQuinticCoefficients(wp0["time"], wp1["time"], wp0["position"][0], wp1["position"][0], 0, 0, 0, 0)
            coeffs_y = self.computeQuinticCoefficients(wp0["time"], wp1["time"], wp0["position"][1], wp1["position"][1], 0, 0, 0, 0)
            coeffs_theta = self.computeQuinticCoefficients(wp0["time"], wp1["time"], wp0["position"][2], wp1["position"][2], 0, 0, 0, 0)

            segment = {
                "coeffs_x": coeffs_x,
                "coeffs_y": coeffs_y,
                "coeffs_theta": coeffs_theta,
                "startTime": wp0["time"],
                "endTime": wp1["time"]
            }
            self.trajectorySegments.append(segment)

    def computeQuinticCoefficients(self, t0, tf, p0, pf, v0, vf, a0, af):
        A = np.array([
            [1, t0, t0**2, t0**3, t0**4, t0**5],
            [1, tf, tf**2, tf**3, tf**4, tf**5],
            [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
            [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
            [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
            [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
        ])
        b = np.array([p0, pf, v0, vf, a0, af])
        return np.linalg.solve(A, b)

    def evaluatePolynomial(self, coeffs, t):
        return np.polyval(coeffs[::-1], t)

    def getNextState(self, dt):
        self.currentTime += dt
        position = np.zeros(3)  # x, y, theta
        velocity = np.zeros(3)

        for segment in self.trajectorySegments:
            if segment["startTime"] <= self.currentTime <= segment["endTime"]:
                t = self.currentTime
                position[0] = self.evaluatePolynomial(segment["coeffs_x"], t)
                position[1] = self.evaluatePolynomial(segment["coeffs_y"], t)
                position[2] = self.evaluatePolynomial(segment["coeffs_theta"], t)

                velocity[0] = self.evaluatePolynomial(np.polyder(segment["coeffs_x"]), t)
                velocity[1] = self.evaluatePolynomial(np.polyder(segment["coeffs_y"]), t)
                velocity[2] = self.evaluatePolynomial(np.polyder(segment["coeffs_theta"]), t)
                return position, velocity

        # After trajectory ends, hold last known position
        if self.trajectorySegments:
            last = self.trajectorySegments[-1]
            t = last["endTime"]
            position[0] = self.evaluatePolynomial(last["coeffs_x"], t)
            position[1] = self.evaluatePolynomial(last["coeffs_y"], t)
            position[2] = self.evaluatePolynomial(last["coeffs_theta"], t)

        return position, velocity
