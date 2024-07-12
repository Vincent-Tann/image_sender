# image_sender.py
import socket
import struct
import pickle
import cv2
import numpy as np
import time
from datetime import datetime
import threading
from copy import deepcopy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from myUtils import draw_posed_3d_box, draw_xyz_axis

class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender')
        self.bridge = CvBridge()
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.rgb_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)
        self.rgb_image = None
        self.depth_image = None
        # self.mask = cv2.imread('path_to_saved_mask.png', cv2.IMREAD_GRAYSCALE)

        self.server_ip = '127.0.0.1'  # 本地服务器IP
        self.server_port = 12345

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.server_ip, self.server_port))
        
        
        # 发送图像
        self.img_fps = 10
        self.send_images_callback_finished = True
        # self.send_timer = self.create_timer(1/self.img_fps, self.send_images) 
        # self.send_timer = threading.Timer(interval=1/self.img_fps, function=self.send_images)
        # self.send_timer.start()
        self.send_idx = 0
        
        
        # FPS计算
        self.num_frames = 0
        # self.time_start = time.time()
        # self.fps_calc_timer = self.create_timer(1, self.fps_calc_timer_callback)
        
        
        # # 接收结果
        # self.pose_receive_finished = True
        self.pose_fps = 30
        # self.receive_pose_timer = self.create_timer(1/self.pose_fps, self.receive_pose_timer_callback)
        # self.receive_pose_timer = threading.Timer(interval=1/self.pose_fps, function=self.receive_pose_timer_callback)
        # self.receive_pose_timer.start()
        
        image_sender_thread = threading.Thread(target=self.image_sender)
        result_receiver_thread = threading.Thread(target=self.result_receiver)
        image_sender_thread.start()
        result_receiver_thread.start()
        image_sender_thread.join()
        result_receiver_thread.join()
    
    def result_receiver(self):
        while True:
            print("receive_pose_timer_callback start!")
            # self.pose_receive_finished = False
            # threading.Timer(interval=1/self.pose_fps, function=self.receive_pose_timer_callback).start()
            result = None
            try:
                # while result is None:
                result = self.sock.recv(1024)  # 接收服务器返回的结果
                result = pickle.loads(result)
            except:
                self.pose_receive_finished = True
                continue
            print(result)
            # current_rgb = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
            # vis = draw_posed_3d_box(result['K'], img=current_rgb, ob_in_cam=result['center_pose'], bbox=result['bbox'])
            # vis = draw_xyz_axis(current_rgb, ob_in_cam=result['center_pose'], scale=0.1, K=result['K'], thickness=3, transparency=0, is_input_rgb=False)
            # cv2.imshow('camera view', vis[...,::-1])
            # cv2.waitKey(1)

            # self.pose_receive_finished = True
    
        
    
    def receive_pose_timer_callback(self):
        # if not self.pose_receive_finished:
        #     return
        print("receive_pose_timer_callback start!")
        self.pose_receive_finished = False
        # threading.Timer(interval=1/self.pose_fps, function=self.receive_pose_timer_callback).start()
        result = None
        try:
            while result is None:
                result = self.sock.recv(1024)  # 接收服务器返回的结果
                result = pickle.loads(result)
        except:
            self.pose_receive_finished = True
            return
        print(result)
        # current_rgb = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
        # vis = draw_posed_3d_box(result['K'], img=current_rgb, ob_in_cam=result['center_pose'], bbox=result['bbox'])
        # vis = draw_xyz_axis(current_rgb, ob_in_cam=result['center_pose'], scale=0.1, K=result['K'], thickness=3, transparency=0, is_input_rgb=False)
        # cv2.imshow('camera view', vis[...,::-1])
        # cv2.waitKey(1)
        
        self.pose_receive_finished = True
    
        
    def fps_calc_timer_callback(self):
        # self.end_timer = time.time()
        print("fps:", self.num_frames)
        self.num_frames = 0
        

    def rgb_callback(self, msg):
        print('rgb update')
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        self.depth_image = (self.depth_image * 1000).astype(np.uint16)  #转化为毫米为单位，uint16
        # print("depth shape:", self.depth_image.shape, ",depth type:", self.depth_image.dtype)
        
    def compress_image(self, image, format='.jpg', quality=80):
        if format == '.jpg':
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        elif format == '.png':
            encode_param = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]  # PNG compression level
        elif format == '.npy':  # For raw NumPy array storage
            return image.tobytes()
        result, encimg = cv2.imencode(format, image, encode_param)
        return encimg.tobytes() if result else None

    def send_images(self):
        # 如果上次调用还未完成，则不执行
        # if not self.send_images_callback_finished:
        #     return
        # 上次已经完成，开启新的调用
        self.send_images_callback_finished = False
        # threading.Timer(interval=1/self.img_fps, function=self.send_images).start()
        if self.rgb_image is not None and self.depth_image is not None:
            try:
                print("--------------------------------------------------------")
                get_image_time = time.time()
                # 获取时间戳
                timestamp = datetime.now().strftime('%Y%m%d%H%M%S%f')
                # 获取图像
                current_rgb = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
                current_depth = deepcopy(self.depth_image)
                # 压缩图像
                compressed_rgb = self.compress_image(current_rgb, format='.jpg')
                compressed_depth = self.compress_image(current_depth, format='.png')
                # compressed_rgb = self.compress_image(self.rgb_image, format='.jpg')
                # compressed_depth = self.compress_image(self.depth_image, format='.png')
                compress_finish_time = time.time()
                print(f"compression time: {compress_finish_time - get_image_time:.4f} seconds")
                current_rgb_byte_size = current_rgb.shape[0] * current_rgb.shape[1] * current_rgb.shape[2] * 8 / 8
                current_depth_byte_size = current_depth.shape[0] * current_depth.shape[1] * 16 / 8
                print(f"rgb size: {len(compressed_rgb)}, compression rate: {len(compressed_rgb)/current_rgb_byte_size*100:.2f}%")
                print(f"depth size: {len(compressed_depth)}, compression rate: {len(compressed_depth)/current_depth_byte_size*100:.2f}%")

                # 定义发送的数据
                data = {'timestamp':timestamp, 'rgb': compressed_rgb, 'depth': compressed_depth}
                data_bytes = pickle.dumps(data)
                # 发送数据大小
                print("sending data size:", len(data_bytes))
                self.sock.sendall(len(data_bytes).to_bytes(4, byteorder='big'))
                # 发送数据
                self.sock.sendall(data_bytes)
                print(f"{self.send_idx} sending finished.")
                self.send_idx +=1
                
                # result = self.sock.recv(1024)  # 接收服务器返回的结果
                # result = pickle.loads(result)
                # print(result)
                receive_pose_time = time.time()
                
                print(f"pose lag: {receive_pose_time - get_image_time :.4f} seconds")
                
                self.num_frames += 1
                
                # vis = draw_posed_3d_box(result['K'], img=current_rgb, ob_in_cam=result['center_pose'], bbox=result['bbox'])
                # vis = draw_xyz_axis(current_rgb, ob_in_cam=result['center_pose'], scale=0.1, K=result['K'], thickness=3, transparency=0, is_input_rgb=False)
                # cv2.imshow('camera view', vis[...,::-1])
                # cv2.waitKey(1)
                
                # cv2.imshow('camera view', cv2.cvtColor(current_rgb, cv2.COLOR_BGR2RGB))
                # cv2.waitKey(1)
                
                print("--------------------------------------------------------")
                    
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
            
            # self.time_end = time.time()
            # fps = self.send_idx / (self.time_end - self.time_start)
            # print(f"fps: {fps:.1f}")
        # 标记调用完成
        self.send_images_callback_finished = True


    def image_sender(self):
        while True:
            if self.rgb_image is not None and self.depth_image is not None:
                try:
                    print("--------------------------------------------------------")
                    get_image_time = time.time()
                    # 获取时间戳
                    timestamp = datetime.now().strftime('%Y%m%d%H%M%S%f')
                    # 获取图像
                    current_rgb = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
                    current_depth = deepcopy(self.depth_image)
                    # 压缩图像
                    compressed_rgb = self.compress_image(current_rgb, format='.jpg')
                    compressed_depth = self.compress_image(current_depth, format='.png')
                    # compressed_rgb = self.compress_image(self.rgb_image, format='.jpg')
                    # compressed_depth = self.compress_image(self.depth_image, format='.png')
                    compress_finish_time = time.time()
                    print(f"compression time: {compress_finish_time - get_image_time:.4f} seconds")
                    current_rgb_byte_size = current_rgb.shape[0] * current_rgb.shape[1] * current_rgb.shape[2] * 8 / 8
                    current_depth_byte_size = current_depth.shape[0] * current_depth.shape[1] * 16 / 8
                    print(f"rgb size: {len(compressed_rgb)}, compression rate: {len(compressed_rgb)/current_rgb_byte_size*100:.2f}%")
                    print(f"depth size: {len(compressed_depth)}, compression rate: {len(compressed_depth)/current_depth_byte_size*100:.2f}%")

                    # 定义发送的数据
                    data = {'timestamp':timestamp, 'rgb': compressed_rgb, 'depth': compressed_depth}
                    data_bytes = pickle.dumps(data)
                    # 发送数据大小
                    print("sending data size:", len(data_bytes))
                    self.sock.sendall(len(data_bytes).to_bytes(4, byteorder='big'))
                    # 发送数据
                    self.sock.sendall(data_bytes)
                    print(f"{self.send_idx} sending finished.")
                    self.send_idx +=1

                    # result = self.sock.recv(1024)  # 接收服务器返回的结果
                    # result = pickle.loads(result)
                    # print(result)
                    receive_pose_time = time.time()

                    print(f"pose lag: {receive_pose_time - get_image_time :.4f} seconds")

                    self.num_frames += 1

                    # vis = draw_posed_3d_box(result['K'], img=current_rgb, ob_in_cam=result['center_pose'], bbox=result['bbox'])
                    # vis = draw_xyz_axis(current_rgb, ob_in_cam=result['center_pose'], scale=0.1, K=result['K'], thickness=3, transparency=0, is_input_rgb=False)
                    # cv2.imshow('camera view', vis[...,::-1])
                    # cv2.waitKey(1)

                    # cv2.imshow('camera view', cv2.cvtColor(current_rgb, cv2.COLOR_BGR2RGB))
                    # cv2.waitKey(1)

                    print("--------------------------------------------------------")

                except Exception as e:
                    self.get_logger().error(f"Error: {e}")

                # self.time_end = time.time()
                # fps = self.send_idx / (self.time_end - self.time_start)
                # print(f"fps: {fps:.1f}")
            # 标记调用完成
            self.send_images_callback_finished = True

def main(args=None):
    rclpy.init(args=args)
    image_sender = ImageSender()
    rclpy.spin(image_sender)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
