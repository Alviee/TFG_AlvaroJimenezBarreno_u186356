from filterpy.kalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
from filterpy.common import Q_discrete_white_noise
import random
import collections

class KFRefined():
    #Se inicializan los valores
    def __init__(self):
        self.f = KalmanFilter(dim_x=2, dim_z=1)
        self.f.x = np.array([2., 0.])
        self.f.F = np.array([[1., 1.],
                        [0., 1.]])
        self.f.H = np.array([[1., 0.]])
        self.f.P = np.array([[1000., 0.],
                        [0., 1000.]])
        self.f.R = np.array([[5.]])

        self.f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)
        self.sensor_detections = collections.deque(maxlen=150)
        self.prev_value = 0.3

    #Funcion para recoger datos en caso que queramos implementar graficos en tiempo real
    def append_list(self, value_list, value):
        value_list.append(value)

    #Prediccion dada una lista de valores
    def predict_values_inlist(self, sensor_list, kalman_instance):

        for i, z in enumerate(sensor_list):
            kalman_instance.predict()
            
            # Outlier detection
            if z > 3 * prev_value:
                prev_value = z
                continue

            self.f.update(z)
            if abs(z - prev_value) > 0.55:
                kalman_instance.x[0] = z
            else:
                kalman_instance.x[0] = (kalman_instance.x[0] + z) / 2

            prev_value = z
        return z

    #Funcion utilizada para el filtrado en tiempo real de los datos.
    def update_with_conditions(self, z, max_range=0.75):
        self.f.predict()
        
        # Outlier detection
        if z > 3 * self.prev_value:
            self.prev_value = z
            return self.f.x[0]

        self.f.update(z)
        if abs(z - self.prev_value) > 0.55:
            self.f.x[0] = z
        else:
            self.f.x[0] = (self.f.x[0] + z) / 2

        self.prev_value = z
        if self.f.x[0] > max_range:
            self.f.x[0] = max_range
        return self.f.x[0]

