

import numpy as np



# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        # State covariance: uncertainty in the estimated robot states.
        # Dimensions: 6x6.
        self.P = np.array(P, dtype=float)

        # Process noise covariance: uncertainty introduced by the motion model.
        # Dimensions: 6x6.
        self.Q = np.array(Q, dtype=float)

        # Measurement noise covariance: uncertainty in IMU and odometry.
        # Dimensions: 4x4.
        self.R = np.array(R, dtype=float)

        # Current state estimate [x, y, theta, w, v, vdot].
        self.x = np.array(x, dtype=float)

        # Time interval between consecutive EKF predictions.
        self.dt = float(dt)
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        # Calculate the Jacobian of the motion model.
        self.A = self.jacobian_A()

        # Calculate the Jacobian of the measurement model.
        self.C = self.jacobian_H()
        
        self.motion_model()
        
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):
        
        # Calculate the Jacobian of the measurement model.
        self.C = self.jacobian_H()

        # Calculate the predicted measurements.
        z_predicted = self.measurement_model()

        # Calculate the measurement innovation (prediction error).
        innovation = np.array(z, dtype=float) - z_predicted

        # Innovation covariance.
        S = np.dot(
            np.dot(self.C, self.P),
            self.C.T
        ) + self.R

        # Kalman gain.
        # solve() avoids explicitly calculating the matrix inverse.
        kalman_gain = np.linalg.solve(
            S,
            np.dot(self.C, self.P)
        ).T

        # Correct the estimated state using the measurement innovation.
        self.x = self.x + np.dot(kalman_gain, innovation)

        # Identity matrix with the same dimensions as the state.
        I = np.eye(len(self.x))

        # Joseph-form covariance update.
        # This is numerically more stable than simply (I - K*C)*P.
        IKC = I - np.dot(kalman_gain, self.C)

        self.P = (
            np.dot(np.dot(IKC, self.P), IKC.T)
            + np.dot(
                np.dot(kalman_gain, self.R),
                kalman_gain.T
            )
        )

        # Remove small numerical asymmetries.
        self.P = 0.5 * (self.P + self.P.T)
        
    
    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,          # Linear velocity (odometry)
            w,          # Angular velocity (odometry)
            vdot,       # Longitudinal acceleration (IMU)
            v * w       # Lateral acceleration (IMU)
        ])

    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt

        self.x = np.array([

            # Update robot position in the global X direction.
            x + v * np.cos(th) * dt,

            # Update robot position in the global Y direction.
            y + v * np.sin(th) * dt,

            # Update robot heading.
            th + w * dt,

            # Angular velocity is assumed constant over dt.
            w,

            # Update linear velocity using longitudinal acceleration.
            v + vdot * dt,

            # Acceleration is assumed constant over dt.
            vdot

        ])


    
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt

        return np.array([

            # x, y, theta, w, v, vdot

            [1, 0, -v * np.sin(th) * dt, 0,
             np.cos(th) * dt, 0],

            [0, 1, v * np.cos(th) * dt, 0,
             np.sin(th) * dt, 0],

            [0, 0, 1, dt, 0, 0],

            [0, 0, 0, 1, 0, 0],

            [0, 0, 0, 0, 1, dt],

            [0, 0, 0, 0, 0, 1]

        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot = self.x

        return np.array([

            # x, y, theta, w, v, vdot

            [0, 0, 0, 0, 1, 0],       # v

            [0, 0, 0, 1, 0, 0],       # w

            [0, 0, 0, 0, 0, 1],       # ax

            [0, 0, 0, v, w, 0]        # ay

        ])
    
    # TODO Part 3: return the states here    
    def get_states(self):
        return self.x.copy()
