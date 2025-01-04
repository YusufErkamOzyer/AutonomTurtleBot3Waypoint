import json
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from math import atan2
import math


class WaypointNavigator(Node):
    def __init__(self, waypoints_file,waypoint_paths_file,task_result_file,distance_odom_file,distance_amcl_file):
        super().__init__('waypoint_navigator')

        # Waypoints dosyasını yükle
        self.waypoints = self.load_waypoints(waypoints_file)
        self.waypoints_file=waypoints_file
        # NavigateToPose action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_listener_callback,
            10
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )


        self.odom_position=""
        self.previous_position = None
        self.odom_current_waypoint_distance=0
        self.odom_total_distance=0


        self.amcl_previous_position= None
        self.amcl_current_waypoint_distance=0
        self.amcl_total_distance=0


        self.distance_amcl_file=distance_amcl_file
        self.amcl_distances=[]


        self.distance_odom_file=distance_odom_file
        self.odom_distances=[]

        

        # Başlangıç parametreleri
        self.current_waypoint_index = 0
        self.feedback_counter = 0 

        self.waypoint_paths_file = waypoint_paths_file
        self.waypoint_paths = []
        self.waypoints_path_index=0

        self.task_result_file=task_result_file
        self.task_results=[]
        self.task_result_index=0


        # Görev döngüsü başlat
        self.start_navigation()
    
    def amcl_callback(self, msg):
        position= msg.pose.pose.position
        
        
        if self.amcl_previous_position == None:
            self.amcl_previous_position=position
            return
        distance=math.sqrt(
            (position.x - self.amcl_previous_position.x) ** 2 +
            (position.y - self.amcl_previous_position.y) ** 2
        )
        self.amcl_current_waypoint_distance=self.amcl_current_waypoint_distance+distance
        self.amcl_total_distance=self.amcl_total_distance+distance

        self.amcl_previous_position=position


    
    
    def odom_listener_callback(self, msg):
        
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        # Quaternion'dan yaw (theta) hesaplama
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        # Quaternion to yaw (theta) dönüşüm formülü
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        theta = atan2(siny_cosp, cosy_cosp)
        self.odom_position=f"{{'x': {position.x:.2f}, 'y': {position.y:.2f}, 'theta': {theta:.2f}}}"


        if self.previous_position==None:
            self.previous_position=position
            return
        
        distance = math.sqrt(
            (position.x - self.previous_position.x) ** 2 +
            (position.y - self.previous_position.y) ** 2
        )
        self.odom_current_waypoint_distance=self.odom_current_waypoint_distance+distance
        self.odom_total_distance=self.odom_total_distance+distance

        self.previous_position = position



















    def load_waypoints(self, file_path):
        """JSON dosyasından waypoint'leri yükle."""
        with open(file_path, 'r') as f:
            return json.load(f)["waypoints"]
        

    def save_distances_odom_data(self,distance_odom_file,odom_distances,odom_total_distance):
        data = {
            "distances_odom": odom_distances,
            "total_distance_odom": odom_total_distance
        }
        with open(distance_odom_file, "w") as outfile:
            json.dump(data, outfile, indent=4)
    
        self.get_logger().info("odom_distances dosyaya ekleme yapildi")
    def save_distances_amcl_data(self,distance_amcl_file,amcl_distinces,amcl_total_distance):
        data = {
            "distances_amcl": amcl_distinces,
            "total_distance_amcl": amcl_total_distance
        }
        with open(distance_amcl_file, "w") as outfile:
            json.dump(data, outfile, indent=4)
    
        self.get_logger().info("amcl_distinces dosyaya ekleme yapildi")




    def save_waypoints_path_data(self,waypoint_paths_file,waypoint_paths):
        with open(waypoint_paths_file, 'w') as outfile:
            json.dump({"waypoint_paths": waypoint_paths}, outfile, indent=4) 
        self.get_logger().info("waypoint_paths dosyaya ekleme yapildi")
    
    def save_task_result_data(self,task_result_file,task_results):
        with open(task_result_file, 'w') as outfile:
            json.dump({"tasks": task_results}, outfile, indent=4) 
        self.get_logger().info("task_results dosyaya ekleme yapildi")




    def start_navigation(self):
        """Waypoints'e sırayla navigasyon başlatır."""
        # Action server'ın hazır olup olmadığını kontrol edin
        while not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Action server bağlanmadı. Bekleniyor...")

        self.get_logger().info("Waypoints navigasyonu başlatılıyor...")
        self.navigate_to_next_waypoint()

    def merged_json_file(self):
        json_files={self.task_result_file,self.distance_amcl_file,self.distance_odom_file,self.waypoint_paths_file,self.waypoints_file}
        merged_data = []

        for file in json_files:
            with open(file, "r", encoding="utf-8") as f:
                data = json.load(f)
                if isinstance(data, list):  
                    merged_data.extend(data)
                else:  
                    merged_data.append(data)

        with open("tour_summary_200202094.json", "w", encoding="utf-8") as f:
            json.dump(merged_data, f, ensure_ascii=False, indent=4)

        print("JSON dosyaları başarıyla birleştirildi: merged.json")


    def navigate_to_next_waypoint(self):
        """Bir sonraki waypoint'e git."""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Bir tur tamamlandı!")
            self.merged_json_file()
            self.current_waypoint_index = 0

        # Şu anki waypoint bilgisi
        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(
            f"{waypoint['id']}. waypoint'e gidiliyor: x={waypoint['x']}, y={waypoint['y']}, w={waypoint['w']}"
        )
        self.feedback_counter = 0 
        # Goal oluşturmerged_json_file
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = waypoint['x']
        goal_pose.pose.position.y = waypoint['y']
        goal_pose.pose.orientation.z = 0.0  # Quaternion z
        goal_pose.pose.orientation.w = waypoint['w']  # Quaternion w

        # NavigateToPose goal mesajı
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Goal gönder
        self._goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        self._goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal reddedildi!")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal kabul edildi. Navigasyon başladı.")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        self.task_result_index +=1
        if result.status == 4:  # SUCCESS
            self.get_logger().info(f"{self.current_waypoint_index + 1}. waypoint başarıyla tamamlandı!")
            #self.get_logger().info(f"from waypoint {self.current_waypoint_index} to waypoint {self.current_waypoint_index + 1} distance: {self.odom_current_waypoint_distance}")
            distance_amcl_data={
                "from_waypoint":self.current_waypoint_index,
                "to_waypoint":self.current_waypoint_index+1,
                "distance":round(self.amcl_current_waypoint_distance, 2)
            }
            total_amcl_distance_data={
                "total_distance":round(self.amcl_total_distance, 2)
            }
            self.amcl_distances.append(distance_amcl_data)
            self.save_distances_amcl_data(self.distance_amcl_file,self.amcl_distances,total_amcl_distance_data)
            self.amcl_current_waypoint_distance=0
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            distance_odom_data={
                "from_waypoint":self.current_waypoint_index,
                "to_waypoint":self.current_waypoint_index+1,
                "distance":round(self.odom_current_waypoint_distance, 2)
            }
            total_distance_data={
                "total_distance":round(self.odom_total_distance, 2)
            }
            self.odom_distances.append(distance_odom_data)
            self.save_distances_odom_data(self.distance_odom_file,self.odom_distances,total_distance_data)

            self.odom_current_waypoint_distance=0

            
            
            
            
            
            data={
                "waypoint_id":self.task_result_index,
                "result":"SUCCESS",
                "current_position":self.odom_position
            }
            self.task_results.append(data)
            self.save_task_result_data(self.task_result_file,self.task_results)
        elif result.status == 5:  # CANCELED
            self.get_logger().warn(f"{self.current_waypoint_index + 1}. waypoint iptal edildi.")
            data={
                "waypoint_id":self.task_result_index,
                "result":"CANCELED",
                "current_position":self.odom_position
            }
            self.task_results.append(data)
            self.save_task_result_data(self.task_result_file,self.task_results)
        elif result.status == 6:  # ABORTED
            self.get_logger().error(f"{self.current_waypoint_index + 1}. waypoint başarısız oldu.")
            data={
                "waypoint_id":self.task_result_index,
                "result":"ABORTED",
                "current_position":self.odom_position
            }
            self.task_results.append(data)
            self.save_task_result_data(self.task_result_file,self.task_results)
        else:
            self.get_logger().error(f"Navigasyon tamamlanamadı. Durum kodu: {result.status}")
            data={
                "waypoint_id":self.task_result_index,
                "result":"UNKNOWN",
                "current_position":self.odom_position
            }
            self.task_results.append(data)
            self.save_task_result_data(self.task_result_file,self.task_results)

        # Sonraki waypoint'e geç
        self.current_waypoint_index += 1
        self.navigate_to_next_waypoint()
    
    
    
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.feedback_counter += 1
        

        # 2. feedback sonrası mesafeyi logla
        if self.feedback_counter == 10:
            if hasattr(feedback, 'distance_remaining'):
                self.get_logger().info(
                    f"Waypoint'e (x={self.waypoints[self.current_waypoint_index]['x']}, y={self.waypoints[self.current_waypoint_index]['y']}) ilk ölçülen hareket mesafesi: {feedback.distance_remaining:.2f} metre"
                )
                self.waypoints_path_index+=1
                data={
                    "waypoint_id":f"{self.waypoints_path_index}",
                    "planned_length":f"{feedback.distance_remaining:.2f} metre"
                }
                self.waypoint_paths.append(data)
                self.save_waypoints_path_data(self.waypoint_paths_file,self.waypoint_paths)

            
            else:
                self.get_logger().warn("Geri bildirimde mesafe bilgisi bulunamadı.")

            



         


def main(args=None):
    rclpy.init(args=args)

    # JSON dosya yolunu belirtin
    waypoints_file = "waypoints_200202094.json"
    waypoints_path_file="path_lengths_200202094.json"
    task_result_file="task_result_200202094.json"
    distance_odom_file="distance_odom_200202094.json"
    distance_amcl_file="distance_amcl_200202094.json"

    # WaypointNavigator node'u başlat
    navigator = WaypointNavigator(waypoints_file,waypoints_path_file,task_result_file,distance_odom_file,distance_amcl_file)

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("Kullanıcı tarafından durduruldu.")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
