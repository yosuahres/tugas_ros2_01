import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import time
import mediapipe as mp

#lib message
from tugas1_msgs.msg import HandResult

class handDetector:
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        self.counter = 0
        self.angle = 0

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(
            static_image_mode=self.mode,
            max_num_hands=self.maxHands,
            min_detection_confidence=self.detectionCon,
            min_tracking_confidence=self.trackCon
        )
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]

    def findHands(self, img, draw=False): 
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(
                        img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=False): 
        xList = []
        yList = []
        zList = []
        bbox = []
        self.lmList = []

        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                cz = lm.z

                xList.append(cx)
                yList.append(cy)
                zList.append(cz)
                self.lmList.append([id, cx, cy, cz])

                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)

            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            bbox = xmin, ymin, xmax, ymax

            if draw:
                cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20),
                              (bbox[2] + 20, bbox[3] + 20), (0, 255, 0), 2)
                cv2.circle(
                    img, (self.lmList[8][1], self.lmList[8][2]), 5, (255, 0, 0), cv2.FILLED)
                cv2.line(img, (self.lmList[0][1] - 200, self.lmList[0][2]),
                         (self.lmList[0][1] + 200, self.lmList[0][2]), (0, 255, 0), 2)
                cv2.line(img, (self.lmList[0][1], self.lmList[0][2]),
                         (self.lmList[4][1], self.lmList[4][2]), (0, 255, 255), 2)

            result = self.calculateAngle(
                (self.lmList[0][1], self.lmList[0][2]), (self.lmList[4][1], self.lmList[4][2]))
            self.angle = result * -1
        return self.lmList, bbox

    def calculateAngle(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        delta_x = x2 - x1
        delta_y = y2 - y1
        angle_rad = math.atan2(delta_y, delta_x)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    def detectHandSide(self, handNo=0):
        if self.results.multi_handedness:
            hand_info = self.results.multi_handedness[handNo]
            hand_label = hand_info.classification[0].label
            return hand_label
        else:
            return ""

    def fingersUp(self):
        fingers = []
        if len(self.lmList) == 0:
            return fingers
        if self.lmList[self.tipIds[0]][1] > self.lmList[self.tipIds[0] - 1][1]:
            fingers.append(1)
        else:
            fingers.append(0)
        for id in range(1, 5):
            if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers

    def findDistance(self, p1, p2, img, draw=False):
        x1, y1 = self.lmList[p1][1], self.lmList[p1][2]
        x2, y2 = self.lmList[p2][1], self.lmList[p2][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        if draw:
            cv2.circle(img, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), 15, (255, 0, 255), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
        length = math.hypot(x2 - x1, y2 - y1)
        return length, img, [x1, y1, x2, y2, cx, cy]

    def isLeft(self):
        if self.angle < 75 and self.angle > 15:
            return True

    def isRight(self):
        if self.angle < 165 and self.angle > 105:
            return True

    def isDown(self):
        if self.lmList:
            if self.isLeft() and self.lmList[0][2] < self.lmList[20][2]:
                if self.lmList[20][2] > self.lmList[19][2]:
                    return True
            elif self.isRight() and self.lmList[0][2] > self.lmList[20][2]:
                if self.lmList[20][2] < self.lmList[19][2]:
                    return True
            elif self.angle >= 65 and self.angle <= 105:
                if self.lmList[20][2] > self.lmList[19][2]:
                    return True

    def isUp(self):
        if self.lmList:
            if self.isLeft():
                if self.lmList[8][2] < self.lmList[5][2]:
                    return True
            elif self.isRight():
                if self.lmList[8][2] < self.lmList[7][2]:
                    return True
            else:
                if self.angle >= 65 and self.angle <= 105:
                    if self.lmList[8][2] < self.lmList[5][2]:
                        return True


class Mediapipe(Node):
    def __init__(self):
        super().__init__('mediapipe')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  
            self.listener_callback,
            10)
        self.subscription  
        self.br = CvBridge()
        self.detector = handDetector()
        self.pTime = 0
        self.get_logger().info('get logger initialized')
        self.hand_result_pub = self.create_publisher(HandResult, 'hand_direction', 10)
        # publish processed image
        self.image_pub = self.create_publisher(Image, 'mediapipe/image', 10)

    def listener_callback(self, data):
        self.get_logger().info('get logger receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        
        img = cv2.flip(current_frame, 1)

        img_with_hands = self.detector.findHands(img, draw=True) 
        lmList, bbox = self.detector.findPosition(img_with_hands, draw=True) 
        handSide = self.detector.detectHandSide()

        # message
        direction = ""
        if self.detector.isUp():
            direction = "up"
        elif self.detector.isDown():
            direction = "down"
        elif self.detector.isLeft():
            direction = "left"
        elif self.detector.isRight():
            direction = "right"
        else:
            direction = "none"

        # publish direction
        msg = HandResult()
        msg.arahtangan = direction
        self.hand_result_pub.publish(msg)

        # publish processed image
        img_msg = self.br.cv2_to_imgmsg(img_with_hands, encoding="bgr8")
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    mediapipe_node = Mediapipe()
    rclpy.spin(mediapipe_node)
    mediapipe_node.destroy_node()
    rclpy.shutdown()
