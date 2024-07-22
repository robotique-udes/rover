import time
import numpy as np
import cv2
import getopt
import sys


class Aruco():
    def __init__(self):
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.samplesize = 4
        self.ids_in_last_frames = [] # Suit le nombre de fois que chaque ID est apparu
        self.validated_ids = [] # Contient les IDs qui sont dans plus que la moitié des frames
        self.list_frames = [] # [0]->Frame le plus récent / [sampleSize-1]->Frame le plus vieux
        # Initialisation des listes
        self.list_frames.clear()
        for i in range(10):
            self.ids_in_last_frames.append(0)
        # Arguments de lancement
        arg_list = sys.argv[1:]
        arg_options = 'hr'
        arg_long_options = ['Help','RSTP stream path']
        
        opts, args = getopt.getopt(arg_list, arg_options, arg_long_options)
        if not args:
            self.rtsp_stream = 0
        else:
            self.rtsp_stream = args[0] 
                


    def empty_list(self,list):
        list.clear()
        return list
    
    def convert_to_gray(self,frame):
        try:
            grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except:
            print('Erreur de conversion en grayscale')
        else:
            return grey_frame

    def draw_on_frame(self,frame,corners,ids): # Fonction optionnelle pour outline le marqueur
        if (len(corners)>0):
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            return frame
        return 0

    def add_frame(self): # Obtiens un frame ici
        
        try:
            cap = cv2.VideoCapture(self.rtsp_stream)
        except:
            print('Erreur de communication avec la caméra, attente de 1 seconde')
            time.sleep(1)
        else:
            ret, frame = cap.read()
            if frame is None:
                print('Image not loaded')
                return 0
            else:
                self.list_frames.insert(0,frame)
                return 1

        # Afficher à l'écran le frame capturé
        # cv2.imshow("Camera feed",frame) 

    def delete_oldest_frame(self):
        ids = self.scan_frame(self.list_frames.pop())
        if (ids is None):
            return 0
        for i in range(len(ids)):
            self.ids_in_last_frames[np.squeeze(ids[i])] -= 1

    def count_valid_ids(self) : # Vérifie si un ID revient assez souvent pour être validé
        for i in range(len(self.ids_in_last_frames)):
            if (self.ids_in_last_frames[i]!=0):
                if ((self.ids_in_last_frames[i]/self.samplesize)>=1):
                    self.validated_ids.append(i)

    def scan_frame(self):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.list_frames[0], 
                                                                   self.aruco_dict, 
                                                                   parameters=self.aruco_params)
        return ids

    def detect_aruco(self):
        while (1):
            if self.add_frame():
                self.frame = self.convert_to_gray(self.frame)
                ids = self.scan_frame()

                if ids is None:
                    print('Pas de marqueurs ArUCo dans le champ de vision')
                else:
                    for i in range(len(ids)):
                        self.ids_in_last_frames[np.squeeze(ids[i])] += 1

                    self.count_valid_ids()

                    print('Les marqueurs ArUCo trouvés sont : ', self.validated_ids, '\n')

                    self.validated_ids = self.empty_list(self.validated_ids)

            
            if (len(self.list_frames)>self.samplesize):
                #print("Deleting")
                self.delete_oldest_frame()


if __name__ == '__main__':
    arucoDetector = Aruco()
    #arucoDetector.detect_aruco()

    

