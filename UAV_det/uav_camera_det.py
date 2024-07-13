# Gerekli Kütüphanelerin İçe Aktarılması
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# YOLOv8 modelinin yüklenmesi
model = YOLO('best.pt')

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, 
            'camera', 
            self.listener_callback, 
            10)
        self.subscription

        self.br = CvBridge()
        self.target_x = None
        self.target_y = None

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')

        # ROS Image mesajını OpenCV görüntüsüne dönüştür
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        image = current_frame

        # Nesne Algılama
        results = model.predict(image)
        img = results[0].plot()  # Tahmin sonuçlarını içeren görüntüyü çiz

        # Tespit edilen nesnelerin koordinatlarını çıkar
        if results[0].boxes:  # Eğer tespit edilen kutular varsa
            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0]  # xyxy koordinatlarını al
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                self.target_x = center_x
                self.target_y = center_y
                self.get_logger().info(f"Object detected at ({center_x}, {center_y})")

                # Takip mekanizmasını burada ekleyin
                self.track_object(center_x, center_y)

        # Sonuçları Göster
        cv2.imshow('Detected Frame', img)
        cv2.waitKey(1)

    def track_object(self, x, y):
        # Takip mekanizmasını burada uygulayın
        # Örneğin, kamerayı veya robotu nesnenin merkezine hareket ettirmek için komutlar gönderin
        self.get_logger().info(f"Tracking object at ({x}, {y})")
        # PID kontrolü, hareket komutları vb. burada uygulanabilir

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
