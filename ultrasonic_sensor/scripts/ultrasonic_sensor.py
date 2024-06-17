#!/usr/bin/env python
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Range
from kalman_filter import SensorDataKalmanFilter
from kalman_filter_refined import KFRefined
import rospy
import serial
import serial.tools.list_ports
import numpy as np

class RangeSensor():

    def __init__(self):
	#Se nicializa los topics y los filtros Kalman para cada sensor.
        self.rear_right_sensor = rospy.Publisher('/keonn/distance_sensor/rear_right_sensor', Range, queue_size=2)
        self.front_right_sensor = rospy.Publisher('/keonn/distance_sensor/front_right_sensor', Range, queue_size=2)
        self.rear_left_sensor = rospy.Publisher('/keonn/distance_sensor/rear_left_sensor', Range, queue_size=2)
        self.front_left_sensor = rospy.Publisher('/keonn/distance_sensor/front_left_sensor', Range, queue_size=2)
        self.kfr_inst_rear_right = KFRefined()
        self.kfr_inst_front_right = KFRefined()
        self.kfr_inst_rear_left = KFRefined()
        self.kfr_inst_front_left = KFRefined()

    def distance_sensor_detection(self):
º	#Se abren los puertos USB para empezar a recibir datos
        rospy.loginfo('Node initialized.')
        rear_right_port = self.open_port(port_name='/dev/ttyUSB_REARRIGHT_DS')
        front_right_port = self.open_port(port_name='/dev/ttyUSB_FRONTRIGHT_DS')
        rear_left_port = self.open_port(port_name='/dev/ttyUSB_REARLEFT_DS')
        front_left_port = self.open_port(port_name='/dev/ttyUSB_FRONTLEFT_DS')

        while not rospy.is_shutdown():
	#Para cada sensor, se crea el mensaje Range y se obtienen los datos directamente filtrados.
            try:
                rear_right_msg = self.create_range_msg('robot_ultrasonic_sensor_rear_right_base_link', 0.74, 0.03, 0.75)
                rear_right_range = self.get_data(rear_right_port, rear_right_msg.max_range, rear_right_msg.min_range) 
                rear_right_msg.range = self.kfr_inst_rear_right.update_with_conditions(rear_right_range)
            except Exception as e:
                rospy.logwarn("Rear right sensor error: " + e.message)
            try:
                front_right_msg = self.create_range_msg('robot_ultrasonic_sensor_front_right_base_link', 0.74, 0.03, 0.75)
                front_right_range = self.get_data(front_right_port, front_right_msg.max_range, front_right_msg.min_range)
                front_right_msg.range = self.kfr_inst_front_right.update_with_conditions(front_right_range)

            except Exception as e:
                rospy.logwarn("Front right sensor error: " + e.message)
            try:
                rear_left_msg = self.create_range_msg('robot_ultrasonic_sensor_rear_left_base_link', 0.92502, 0.03, 0.75)
                rear_left_range = self.get_data(rear_left_port, rear_right_msg.max_range, rear_right_msg.min_range) 
                rear_left_msg.range = self.kfr_inst_rear_left.update_with_conditions(rear_left_range)

            except Exception as e:
                rospy.logwarn("Rear left sensor error: " + e.message)
            try:       
                front_left_msg = self.create_range_msg('robot_ultrasonic_sensor_front_left_base_link', 0.92502, 0.03, 0.75)
                front_left_range = self.get_data(front_left_port, front_right_msg.max_range, front_right_msg.min_range)
                front_left_msg.range = self.kfr_inst_front_left.update_with_conditions(front_left_range)

            except Exception as e:
                rospy.logwarn("Front left sensor error: " + e.message)

            self.rear_right_sensor.publish(rear_right_msg)
            self.front_right_sensor.publish(front_right_msg)
            self.rear_left_sensor.publish(rear_left_msg)
            self.front_left_sensor.publish(front_left_msg)

    #Funcion de creacion de mensaje Range
    def create_range_msg(self, frame, fov, min_range, max_range):
        range_name = Range()
        range_name.header.stamp = rospy.Time.now()
        range_name.header.frame_id = frame
        range_name.radiation_type = Range.ULTRASOUND
        range_name.field_of_view = fov
        range_name.min_range = min_range
        range_name.max_range = max_range
        return range_name

    #Funcion de lectura de datos
    def read_sensor_line(self, ser):
        eol = b'\r'
        leneol = len(eol)
        line = bytearray()
        fbyte = False
        while True:
            first_byte = True
            c = ser.read(1)
            if not fbyte:
                fbyte = True
                continue
            if c and c !=b'R':
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)

    #Funcion de apertura de puertos. Se setean todas las variables
    def open_port(self, port_name):
        ser = serial.Serial()
        ser.port = port_name
        ser.baudrate = 9600
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.timeout = 0.1
        ser.open()
        return ser

    #Funcion de conversion de datos
    def get_data(self, ser, max_range, min_range):
        sensor_line = self.read_sensor_line(ser)
        sensor_line = sensor_line.decode('utf-8')
        data = float(sensor_line) / 1000.0
        if data >= max_range:
            data = max_range
        elif data <= min_range:
            data = min_range
        return data

    #Funcion para chequear los puertos abiertos.
    def port_routes(self):
        ports = serial.tools.list_ports.comports()
        
        for p in ports:
            print("Puerto: " + str(p.device))

if __name__=='__main__':
    try:
        #port_routes()
        rospy.init_node('ultrasonic_sensor', log_level=rospy.DEBUG)
        range = RangeSensor()
        range.distance_sensor_detection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
