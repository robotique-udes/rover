#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rover_msgs.msg import UsbData
import subprocess
import os
import time
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


class USBParserNode(Node):
    def __init__(self):
        super().__init__('usb_parser_node')
        self.publisher_ = self.create_publisher(UsbData, 'usb_parsing_result', 10)
        self.create_timer(5.0, self.timer_callback)

        self.liste_reference_ports = self.info_ports()
        self.no_clef = 1
        self.last_published_data = None
        self.last_usb_id = None
        self.usb_connected = False
        self.lastMessage = ("", "")
        
        package_share_directory = get_package_share_directory("rover_navigation")
        self.saved_locations_path = package_share_directory + "/../../../../src/rover/rover_navigation/saved_files/saved_documents.json"

        self.publish_empty_data()

    def publish_empty_data(self):
        msg = UsbData()
        msg.timestamp = ""
        msg.data = ""
        self.publisher_.publish(msg)

    def timer_callback(self):
        result_verif = self.verification_usb(self.liste_reference_ports)
        clef_inseree = result_verif[0]
        idVendor_clefusb = result_verif[1]
        idProduct_clefusb = result_verif[2]

        if clef_inseree:
            current_usb_id = f"{idVendor_clefusb}:{idProduct_clefusb}"
            return_parsing = self.parsing(self.no_clef)
            message_changement_cellule, message_log, log = return_parsing
            
            if not self.usb_connected or self.last_usb_id != current_usb_id:
                msg = UsbData()
                msg.timestamp = self.get_current_timestamp()
                msg.data = message_log
                self.publisher_.publish(msg)
                self.lastMessage = (msg.timestamp, msg.data)
                self.last_usb_id = current_usb_id
                self.get_logger().info("USB data published with new timestamp: %s" % msg.timestamp)
                self.no_clef += 1
            else:
                if (message_changement_cellule, message_log) != self.lastMessage:
                    msg = UsbData()
                    msg.timestamp = self.lastMessage[0]
                    msg.data = message_log
                    self.publisher_.publish(msg)
                    self.lastMessage = (msg.timestamp, msg.data)
                    self.get_logger().info("Republished data with same timestamp: %s" % msg.timestamp)

            self.usb_connected = True
        else:
            msg = UsbData()
            msg.timestamp = self.lastMessage[0]
            msg.data = self.lastMessage[1]
            self.publisher_.publish(msg)
            self.get_logger().info("Published last known data with timestamp: %s" % msg.timestamp)
            self.usb_connected = False
            
    def get_current_timestamp(self):
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    def info_ports(self):
        info_ports = subprocess.Popen(["lsusb"], stdout=subprocess.PIPE)
        communication_ports = info_ports.communicate()[0]
        devices_initiaux = communication_ports.decode("utf-8").splitlines()

        liste_devices = []
        for a_device in devices_initiaux:
            liste_pour_un_device = a_device.split()
            ses_id = liste_pour_un_device[5].split(":")
            reference_idVendor = ses_id[0]
            reference_idProduct = ses_id[1]
            ids = [reference_idVendor, reference_idProduct]
            liste_devices.append(ids)
        return liste_devices

    def parsing(self, no_clef):
        self.get_logger().info(f"Starting parsing for USB key {no_clef}")
        time.sleep(3)
        pathway = None

        mount_point = "/media"

        for root, dirs, files in os.walk(mount_point):
            self.get_logger().debug(f"Searching in directory: {root}")
            if "lander.log" in files:
                pathway = os.path.join(root, "lander.log")
                self.get_logger().info(f"Found lander.log at: {pathway}")
                break

        if pathway is not None:
            try:
                with open(pathway, "r") as fichier:
                    lignes_fichier = fichier.read()
                    
                self.writeDoc(self, lignes_fichier)
                
                message_changement_cellule = "Cells status update"
                message_log = lignes_fichier
                log = lignes_fichier

                return message_changement_cellule, message_log, log
            except Exception as e:
                
                self.get_logger().error(f"Error reading or parsing file: {str(e)}")
                return "Error: Failed to read or parse USB data", "File read error", str(e)
        else:
            self.get_logger().warn("lander.log not found in any mounted USB device")
            return "Error: lander.log not found", "USB key content error", "lander.log could not be found on the USB key"

    def writeDoc(self, lignes_ficher):
        with open(self.saved_locations_path, "w") as fichier:
            fichier.write(lignes_ficher)
            
        #TODO
        #VERIFY LOGIC HERE WITH USB KEY
        
    def verification_usb(self, liste_reference_ports):
        actual_liste_devices = self.info_ports()
        idVendor_clefusb = "N/A"
        idProduct_clefusb = "N/A"
        usb_connecte = False
        new_usb = True

        for device in actual_liste_devices:
            device_idVendor, device_idProduct = device
            if [device_idVendor, device_idProduct] not in liste_reference_ports:
                new_usb = True
                idVendor_clefusb = device_idVendor
                idProduct_clefusb = device_idProduct
                usb_connecte = True
                break

        return usb_connecte, idVendor_clefusb, idProduct_clefusb

def main(args=None):
    rclpy.init(args=args)
    node = USBParserNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()