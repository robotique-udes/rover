import rclpy
from rclpy.node import Node
import time
import numpy as np
import cv2
import getopt
import sys

from rover_msgs.msg import Aruco as ArucoMsg

class Aruco(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.aruco_pub = self.create_publisher(ArucoMsg, '/rover/auxiliary/aruco', 1)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.samplesize = 4
        self.ids_in_last_frames = []  # Suit le nombre de fois que chaque ID est apparu
        self.validated_ids = []  # Contient les IDs qui sont dans plus que la moitié des frames
        self.list_frames = []  # [0]->Frame le plus récent / [sampleSize-1]->Frame le plus vieux
        self.frame = None  # Initialize frame attribute
        # Initialisation des listes
        self.list_frames.clear()
        # Arguments de lancement
        self.rtsp_stream = '/dev/v4l/by-id/usb-046d_0825_E861A1D0-video-index0'

    def empty_list(self, list):
        list.clear()
        return list

    def convert_to_gray(self, frame):
        try:
            grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except:
            print('Erreur de conversion en grayscale')
            return None
        else:
            return grey_frame

    def draw_on_frame(self, frame, corners, ids):  # Fonction optionnelle pour outline le marqueur
        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            return frame
        return 0

    def add_frame(self):  # Obtiens un frame ici
        try:
            cap = cv2.VideoCapture(self.rtsp_stream)
        except:
            print('Erreur de communication avec la caméra, attente de 1 seconde')
            time.sleep(1)
            return 0
        else:
            ret, frame = cap.read()
            if frame is None:
                print('Image not loaded')
                return 0
            else:
                self.frame = frame  # Assign the frame to self.frame
                self.list_frames.insert(0, frame)
                return 1

    def delete_oldest_frame(self):
        if len(self.list_frames) > 0:
            ids = self.scan_frame(self.list_frames.pop())
            if ids is None:
                return 0
            for i in range(len(ids)):
                id = np.squeeze(ids[i])
                if id < len(self.ids_in_last_frames):
                    self.ids_in_last_frames[id] -= 1

    def count_valid_ids(self):  # Vérifie si un ID revient assez souvent pour être validé
        for i in range(len(self.ids_in_last_frames)):
            if self.ids_in_last_frames[i] != 0:
                if (self.ids_in_last_frames[i] / self.samplesize) >= 1:
                    self.validated_ids.append(i)

    def scan_frame(self, frame):
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        return ids

    def detect_aruco(self):
        while True:
            if self.add_frame():
                self.frame = self.convert_to_gray(self.frame)
                if self.frame is not None:
                    ids = self.scan_frame(self.frame)
                    if ids is None:
                        print('Pas de marqueurs ArUCo dans le champ de vision')
                    else:
                        for i in range(len(ids)):
                            id = np.squeeze(ids[i])
                            if id >= len(self.ids_in_last_frames):
                                self.ids_in_last_frames.extend([0] * (id - len(self.ids_in_last_frames) + 1))
                            self.ids_in_last_frames[id] += 1

                        self.count_valid_ids()
                        
                        msg = ArucoMsg()
                        msg.valid = True
                        msg.id = self.validated_ids
                        self.aruco_pub.publish(msg)
                        # print('Les marqueurs ArUCo trouvés sont : ', self.validated_ids, '\n')
                        self.validated_ids = self.empty_list(self.validated_ids)

            if len(self.list_frames) > self.samplesize:
                self.delete_oldest_frame()

if __name__ == '__main__':
    try:
        rclpy.init(args=None)
        aruco_detector = Aruco()
        rclpy.spin(aruco_detector)
        aruco_detector.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
