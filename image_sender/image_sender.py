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

class ImageSenderNode(Node):
    def __init__(self, sock):
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
        self.sock = sock
        
        
        # 发送图像
        self.img_fps = 15
        # self.send_images_callback_finished = True
        self.send_timer = self.create_timer(1/self.img_fps, self.send_images) 
        # self.send_timer = threading.Timer(interval=1/self.img_fps, function=self.send_images)
        # self.send_timer.start()
        self.send_idx = 0
        
        
        # FPS计算
        self.num_frames = 0
        # self.time_start = time.time()
        # self.fps_calc_timer = self.create_timer(1, self.fps_calc_timer_callback)
        
        
    def fps_calc_timer_callback(self):
        # self.end_timer = time.time()
        print("fps:", self.num_frames)
        self.num_frames = 0
        

    def rgb_callback(self, msg):
        print('rgb update')
        self.timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        self.depth_image = (self.depth_image * 1000).astype(np.uint16)  #转化为毫米为单位，uint16
        # print("depth shape:", self.depth_image.shape, ",depth type:", self.depth_image.dtype)
        
    def compress_image(self, image, format='.jpg', quality=70):
        if format == '.jpg':
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        elif format == '.png':
            encode_param = [int(cv2.IMWRITE_PNG_COMPRESSION), 5]  # PNG compression level
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
                # timestamp = datetime.now().strftime('%Y%m%d%H%M%S%f')
                timestamp = deepcopy(self.timestamp)
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
                print(f"rgb size: {len(compressed_rgb)}, compression rate: {len(compressed_rgb)/len(current_rgb.tobytes())*100:.2f}%")
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

                # receive_pose_time = time.time()
                
                # print(f"pose lag: {receive_pose_time - get_image_time :.4f} seconds")
                
                self.num_frames += 1
                
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


class PoseReceiverNode(Node):
    def __init__(self, sock, image_sender_node:ImageSenderNode):
        super().__init__('pose_receiver')
        self.sock = sock
        
        self.bridge = CvBridge()
        # self.rgb_subscription = self.create_subscription(
        #     Image,
        #     '/camera/image_raw',
        #     self.rgb_callback,
        #     10)
        
        self.image_sender_node = image_sender_node
        
        self.rgb_image = None
        
        receive_fps = 15
        self.receive_timer = self.create_timer(1/receive_fps, self.receive_pose_timer_callback)
        
        # FPS计算
        self.num_receive_frames = 0
        self.receive_fps = 0
        self.time_start = time.time()
        self.fps_calc_timer = self.create_timer(1, self.fps_calc_timer_callback)
        
        self.rgb_idx = 0
        
    # def rgb_callback(self, msg):
    #     print('[rgb_callback] callback function triggered, self.rgb_image updated! rgb_idx =', self.rgb_idx)
    #     self.rgb_idx +=1
    #     self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #     if self.rgb_image is not None:
    #         cv2.imshow('camera view', self.rgb_image)
    #         cv2.waitKey(0)

    def receive_pose_timer_callback(self):
        # if not self.pose_receive_finished:
        #     return
        # print("receive_pose_timer_callback start!")
        # self.pose_receive_finished = False
        # threading.Timer(interval=1/self.pose_fps, function=self.receive_pose_timer_callback).start()
        result = None
        try:
            # while result is None:
            result = self.sock.recv(1024)  # 接收服务器返回的结果
            result = pickle.loads(result)
            self.num_receive_frames += 1
        except:
            # self.pose_receive_finished = True
            return
        # print(result)
        # return
        # if self.image_sender_node.rgb_image is not None:
        #     current_rgb = cv2.cvtColor(self.image_sender_node.rgb_image, cv2.COLOR_BGR2RGB)
        #     vis = draw_posed_3d_box(result['K'], img=current_rgb, ob_in_cam=result['center_pose'], bbox=result['bbox'])
        #     vis = draw_xyz_axis(current_rgb, ob_in_cam=result['center_pose'], scale=0.1, K=result['K'], thickness=3, transparency=0, is_input_rgb=False)
        #     cv2.imshow('camera view', vis[...,::-1])
        #     cv2.waitKey(1)
        if self.image_sender_node.rgb_image is not None:
            current_timestamp = self.image_sender_node.timestamp
            current_rgb = deepcopy(self.image_sender_node.rgb_image)
            pose_timestamp = result['timestamp']
            delta_time = datetime.strptime(current_timestamp, '%Y-%m-%d %H:%M:%S.%f') - datetime.strptime(pose_timestamp, '%Y-%m-%d %H:%M:%S.%f')
            delta_sec, delta_mic = delta_time.seconds, delta_time.microseconds
            vis = draw_posed_3d_box(result['K'], img=current_rgb, ob_in_cam=result['center_pose'], bbox=result['bbox'])
            vis = draw_xyz_axis(current_rgb, ob_in_cam=result['center_pose'], scale=0.1, K=result['K'], thickness=3, transparency=0, is_input_rgb=False)
            text = ""
            vis = cv2.putText(vis, f"image timestamp: {current_timestamp}", (10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,0,0))
            vis = cv2.putText(vis, f"pose  timestamp: {result['timestamp']}", (10,40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0,255,0))
            vis = cv2.putText(vis, f"time lag: {delta_sec}.{delta_mic} seconds", (10,60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0,0,255))
            vis = cv2.putText(vis, f"received pose rate: {self.receive_fps}", (10,80), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255))
            cv2.imshow('camera view', vis)
            cv2.waitKey(1)
     
        # self.pose_receive_finished = True
        
    def fps_calc_timer_callback(self):
        self.receive_fps = self.num_receive_frames * 1.0
        self.num_receive_frames = 0
        
    
    
    
    
from rclpy.executors import MultiThreadedExecutor
import threading

def main(args=None):
    rclpy.init(args=args)
    
    server_ip = '127.0.0.1'  # 本地服务器IP
    server_port = 12345

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_ip, server_port))

    # 创建节点
    send_image_node = ImageSenderNode(sock)
    receive_pose_node = PoseReceiverNode(sock, send_image_node)

    # 创建executor
    executor = MultiThreadedExecutor()

    # 将节点添加到executor
    executor.add_node(send_image_node)
    executor.add_node(receive_pose_node)

    # 创建线程
    thread = threading.Thread(target=executor.spin)

    # 启动线程
    thread.start()

    # 等待线程完成
    thread.join()

    # 销毁节点
    send_image_node.destroy_node()
    receive_pose_node.destroy_node()

    rclpy.shutdown()


# def main(args=None):
#     rclpy.init(args=args)
#     image_sender = ImageSenderNode()
#     rclpy.spin(image_sender)
#     rclpy.shutdown()

if __name__ == '__main__':
    main()
