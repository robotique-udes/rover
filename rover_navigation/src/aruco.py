import time
import numpy as np
import cv2



class Aruco() :
    def __init__(self) :
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.videonumber = 0 # Changer selon le numéro de feed vidéo
        self.sampleSize = 4
        self.iDsInLastFrames = [] # Suit le nombre de fois que chaque ID est apparu
        self.validatedIDs = [] # Contient les IDs qui sont dans plus que la moitié des frames
        self.listFrames = [] # [0]->Frame le plus récent / [sampleSize-1]->Frame le plus vieux
        #Initialisation des listes
        self.listFrames.clear()
        for i in range(10):
            self.iDsInLastFrames.append(0)

    def setEmptyList(self,list) :
        list.clear()
        return list

    def drawOnFrame(self,frame,corners,ids) : # Fonction optionnelle (?) pour avoir une outline sur le marqueur
        if (len(corners)>0) :
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            return frame
        return 0

    def AddFrame(self) : # Obtiens un frame ici
        
        cap = cv2.VideoCapture(self.videonumber)
        assert cap.isOpened()
        ret, frame = cap.read()

        # Afficher à l'écran le frame capturé
        # cv2.imshow("Camera feed",frame) 
        
        if frame is None:
            print("Image not loaded")
            return 0
        else :
            self.listFrames.insert(0,frame)
            return 1

    def deleteOldestFrame(self):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.listFrames.pop(), self.arucoDict, parameters=self.arucoParams)
        if (ids is None):
            return 0
        for i in range(len(ids)) :
            self.iDsInLastFrames[np.squeeze(ids[i])] -= 1


    def countValidIDs(self) : # Vérifie si un ID revient assez souvent pour être validé
        for i in range(len(self.iDsInLastFrames)) :
            if (self.iDsInLastFrames[i]!=0) :
                if ((self.iDsInLastFrames[i]/self.sampleSize)>=1) :
                    self.validatedIDs.append(i)

    def detectAruco(self):
        while (1) :
            if (self.AddFrame()):
                (corners, ids, rejected) = cv2.aruco.detectMarkers(self.listFrames[0], self.arucoDict, parameters=self.arucoParams)

            if (ids is None):
                print("Pas de marqueurs ArUCo dans le champ de vision")
            else:
                for i in range(len(ids)) :
                    self.iDsInLastFrames[np.squeeze(ids[i])] += 1

                self.countValidIDs()

                print("The detected ArUCo markers are : ", self.validatedIDs, "\n")

                self.validatedIDs = self.setEmptyList(self.validatedIDs)
            
            if (len(self.listFrames)>self.sampleSize) :
                #print("Deleting")
                self.deleteOldestFrame()


def main() :
    arucoDetector = Aruco()
    arucoDetector.detectAruco()

if True :
    main()
    

