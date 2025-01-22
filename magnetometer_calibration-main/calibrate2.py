import numpy as np
from scipy import linalg
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Magnetometer class
class Magnetometer(object):
    MField = 1000  # Gravitation Field (adjust according to your environment)

    def __init__(self, F=MField):
        self.F = F
        self.b = np.zeros([3, 1])
        self.A_1 = np.eye(3)

    def run(self):
        data = self.load_data("mag.log")
        print("shape of data:", data.shape)
        print("First 5 rows raw:\n", data[:5])

        # Ellipsoid fit
        s = np.array(data).T
        M, n, d = self.__ellipsoid_fit(s)

        # Calibration parameters
        M_1 = linalg.inv(M)
        self.b = -np.dot(M_1, n)
        self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))

        print("Soft iron transformation matrix:\n", self.A_1)
        print("Hard iron bias:\n", self.b)

        # Apply calibration
        calibrated_data = []
        for row in data:
            # Subtract hard iron offset
            xm_off = row[0] - self.b[0]
            ym_off = row[1] - self.b[1]
            zm_off = row[2] - self.b[2]

            # Multiply by the inverse soft iron offset
            xm_cal = xm_off * self.A_1[0, 0] + ym_off * self.A_1[0, 1] + zm_off * self.A_1[0, 2]
            ym_cal = xm_off * self.A_1[1, 0] + ym_off * self.A_1[1, 1] + zm_off * self.A_1[1, 2]
            zm_cal = xm_off * self.A_1[2, 0] + ym_off * self.A_1[2, 1] + zm_off * self.A_1[2, 2]

            # calibrated_data.append([xm_cal, ym_cal, zm_cal])
            calibrated_data.append([xm_off, ym_off, zm_off])

        calibrated_data = np.array(calibrated_data)

        # Plot raw and calibrated data
        fig = plt.figure(figsize=(12, 6))

        # Raw data plot
        ax1 = fig.add_subplot(121, projection='3d')
        ax1.scatter(data[:, 0], data[:, 1], data[:, 2], color='r', marker='o', label='Raw Data')
        ax1.set_title('Raw Magnetometer Data')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        ax1.legend()

        # Calibrated data plot
        ax2 = fig.add_subplot(122, projection='3d')
        ax2.scatter(calibrated_data[:, 0], calibrated_data[:, 1], calibrated_data[:, 2], color='g', marker='o', label='Calibrated Data')
        ax2.set_title('Calibrated Magnetometer Data')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')
        ax2.legend()

        plt.show()

        print("First 5 rows calibrated:\n", calibrated_data[:5])
        np.savetxt('out.txt', calibrated_data, fmt='%f', delimiter=',')

    def load_data(self, filename):
        data = []
        with open(filename, 'r') as file:
            for line in file:
                parts = line.strip().split()
                if len(parts) == 4:
                    _, x, y, z = parts
                    data.append([float(x), float(y), float(z)])
        return np.array(data)

    def __ellipsoid_fit(self, s):
        D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                      2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                      2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

        S = np.dot(D, D.T)
        S_11 = S[:6, :6]
        S_12 = S[:6, 6:]
        S_21 = S[6:, :6]
        S_22 = S[6:, 6:]

        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -4,  0,  0],
                      [ 0,  0,  0,  0, -4,  0],
                      [ 0,  0,  0,  0,  0, -4]])

        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)
        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: 
            v_1 = -v_1

        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        M = np.array([[v_1[0], v_1[5], v_1[4]],
                      [v_1[5], v_1[1], v_1[3]],
                      [v_1[4], v_1[3], v_1[2]]])
        n = np.array([[v_2[0]],
                      [v_2[1]],
                      [v_2[2]]])
        d = v_2[3]

        return M, n, d

if __name__ == '__main__':
    Magnetometer().run()
