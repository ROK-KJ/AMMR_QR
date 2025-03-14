#!/usr/bin/env python3

import cv2
import numpy as np
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from arucomarker.msg import MarkerDetect, MarkerDetectArray
from rclpy.qos import QoSProfile
from scipy.spatial.transform import Rotation as R

class MarkerDetectEstimate(Node):
    def __init__(self):
        super().__init__('MarkerDetecter')  # node 초기화

        # opencv-ros 변환 브리지 생성
        self.bridge = CvBridge()

        # 필요한 인자들 초기화
        self.ZedCvImage = None
        self.DepthImage = None
        self.UndistortedImage = None

        self.Coners = None
        self.Ids = None
        self.Rvecs = None
        self.Tvecs = None
        self.st_IntrinsicParameter = np.array([[]])
        self.st_DistortionParameter = np.zeros((8,), dtype=np.float32)

        self.MarkerSize = 0.05 # 단위 m
        
        # Publish
        self.st_PubMsg = self.create_publisher(
            MarkerDetectArray,
            '/TestPubArUco',
            QoSProfile(depth=10)
        )

        # ZED Depth Sub
        # self.st_SubZEDDepth = self.create_subscription(
            # Image,
            # '/zed/zed_node/depth/depth_registered',
            # self.ZEDDepthCallBack,
            # 10
        # )

        # ZEE Camera Image Sub
        self.st_SubZEDImage = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.ZEDImageCallBack,
            10
        )

        # ZED Camera Intrinsic Parameter, Distortion Parameter Sub
        self.st_SubZEDParams = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self.ZEDParamsCallBack,
            10
        )

        # ArUco 검출기 설정
        self.Dict_AruCo = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.AruCoParams = cv2.aruco.DetectorParameters() 
        self.ArucoDetector = cv2.aruco.ArucoDetector(self.Dict_AruCo, self.AruCoParams)
        
        ########### AruCoParams, Dict_AruCo 출력 ###################
        # for attr in dir(self.AruCoParams):
            # if not attr.startswith("__"):  # 특수 메서드 제외
                # print(f"{attr}: {getattr(self.AruCoParams, attr)}")

        # for attr in dir(self.Dict_AruCo):
            # if not attr.startswith("__"):  # 특수 메서드 제외
                # print(f"{attr}: {getattr(self.Dict_AruCo, attr)}")
        
        #############################################################
            
        while self.ZedCvImage is None:
            print("waiting the Image")
            rclpy.spin_once(self) # 이미지가 들어올 때까지 대기

        print("Image uploaded success!")

    # def ZEDDepthCallBack(self, DepthMsg):
    #     self.DepthImage = self.bridge.imgmsg_to_cv2(DepthMsg, desired_encoding='passthrough')
    
    def ZEDImageCallBack(self, ImageMsg):
        self.ZedCvImage = self.bridge.imgmsg_to_cv2(ImageMsg, desired_encoding="bgr8")
        
        self.MainProcessing()

        # cv2.imshow("ZED Image", self.ZedCvImage)
        # cv2.waitKey(1)  

    def ZEDParamsCallBack(self, ParamsMsg):
        # print("ZEDParamsCallBack")
        # print("\n------------ParamsMsg-----------------")
        self.st_IntrinsicParameter = np.array(ParamsMsg.k, dtype=np.float32).reshape(3, 3)
        self.st_DistortionParameter = np.array(ParamsMsg.d, dtype=np.float32).flatten()

        # print(self.st_IntrinsicParameter)
        # print(self.st_DistortionParameter)
        # print("-----------------------------------")

    def DetectAndPose(self):
        # ArUco Marker 검출 및 3D 포즈 추정
        # print("\n-----------------------------------")
        # print("DetectAndPose")

        # 왜곡 보정해서 넘어온 frame으로
        self.Coners, self.Ids, _ = self.ArucoDetector.detectMarkers(self.UndistortedImage)
        # print("Coners")
        # print(self.Coners)
        # print("Ids")
        # print(self.Ids)

        if self.Ids is not None: # Ids가 존재하면
            cv2.aruco.drawDetectedMarkers(self.UndistortedImage, self.Coners, self.Ids)
            
            self.Rvecs, self.Tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                self.Coners, self.MarkerSize, self.st_IntrinsicParameter, self.st_DistortionParameter
            )
        # print("self.Rvecs")
        # print(self.Rvecs)
        
        # print("self.Tvecs")
        # print(self.Tvecs)
        
        # print("-----------------------------------")    
            
    def DisplayMarkerInfo(self):
        # ArUco Marker Info Draw
        print("DisplayMarkerInfo")

        self.PubMsg = MarkerDetectArray()

        for i in range(len(self.Ids)):
            st_Marker = MarkerDetect() 

            st_Marker.id = int(self.Ids[i][0])  

            st_Marker.position.x = float(self.Tvecs[i][0][0])
            st_Marker.position.y = float(self.Tvecs[i][0][1])
            st_Marker.position.z = float(self.Tvecs[i][0][2])

            corner = self.Coners[i][0]
            center_x = int(np.mean(corner[:, 0]))
            center_y = int(np.mean(corner[:, 1]))
            
            pos_x, pos_y, pos_z = self.Tvecs[i][0]
            RotationMatrix, _ = cv2.Rodrigues(self.Rvecs[i])
            
            EulerAngles = cv2.RQDecomp3x3(RotationMatrix)[0]
            
            print("EulerAngles")
            print(EulerAngles)
            
            st_Marker.roll = EulerAngles[2]
            st_Marker.pitch = EulerAngles[0]
            st_Marker.yaw = EulerAngles[1]

            self.PubMsg.markerarray.append(st_Marker)
            
            # cv2.drawFrameAxes(self.UndistortedImage, self.st_IntrinsicParameter, self.st_DistortionParameter,
                            # self.Rvecs[i], self.Tvecs[i], self.MarkerSize / 2)

            # cv2.putText(self.UndistortedImage, f"ID: {self.Ids[i][0]}", (center_x, center_y - 40),
                        # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            # cv2.putText(self.UndistortedImage, f"Pos: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})m",
                        # (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            # cv2.putText(self.UndistortedImage, f"Rot: ({EulerAngles[0]:.1f}, {EulerAngles[1]:.1f}, {EulerAngles[2]:.1f})deg",
                        # (center_x, center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            # cv2.putText(self.UndistortedImage, f"Depth: {self.DepthValue:.2f}m",
                        # (center_x, center_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    def MainProcessing(self):

        # 왜곡 보정 진행
        self.UndistortedImage = cv2.undistort(self.ZedCvImage, self.st_IntrinsicParameter, self.st_DistortionParameter)

        # ArUco 마커 검출 및 3D 포즈 추정
        self.DetectAndPose()

        if self.Ids is not None:
            self.DisplayMarkerInfo()
            self.st_PubMsg.publish(self.PubMsg)
            
        cv2.imshow("Final Image", self.UndistortedImage)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    RosNode = MarkerDetectEstimate()
    
    rclpy.spin(RosNode)  # 노드 실행
    RosNode.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()