import math
from topological_mapping import HybridMap
# A* yerine Greedy kullanıyoruz çünkü hedefin konumu bilindiğinde daha verimli
from greedy_algorithm import GreedyBestFirst
from obstacle_detector import ObstacleDetector
import numpy as np

# Navigasyon sistemi - tüm bileşenleri entegre eden ana modül
# Yapılandırılmamış ortamlar için hibrit yol planlama (Liu vd. [17])
class NavigationSystem:
    def __init__(self, width, height, resolution=0.1):
        # Haritalama ve yol planlama bileşenlerini başlat
        self.hybrid_map = HybridMap(width, height, resolution)
        self.path_planner = GreedyBestFirst(self.hybrid_map)
        self.obstacle_detector = ObstacleDetector()
        
        # Navigasyon durumu
        self.current_path = None
        self.current_position = (0, 0)  # metre cinsinden (x, y)
        self.current_orientation = 0.0  # radyan
        self.goal_position = None
        
        # Yol takip parametreleri
        self.waypoint_threshold = 0.2  # metre, hedefe ulaşma mesafesi eşiği
        self.obstacle_threshold = 0.5  # metre, engel algılama mesafesi eşiği
        
        # İlk topolojik node'u oluştur (daha sonra dinamik olarak güncellenebilir)
        self.start_node_id = self.hybrid_map.add_node(0, 0)
    
    def set_goal(self, x, y):
        # Yeni bir navigasyon hedefi belirle
        self.goal_position = (x, y)
        
        # Hedef için topolojik node oluştur (yoksa)
        nearest_id, distance = self.hybrid_map.find_nearest_node(x, y)
        
        if distance > 1.0:  # Hedef mevcut nodelerden uzaksa
            goal_node_id = self.hybrid_map.add_node(x, y)
            
            # Bağlantılılığı sağlamak için en yakın nodee bağla
            if nearest_id is not None:
                self.hybrid_map.connect_nodes(nearest_id, goal_node_id)
            
            return goal_node_id
        else:
            return nearest_id
    
    def update_position(self, x, y, orientation):
        # Robotun mevcut konum ve yönelimini güncelle
        self.current_position = (x, y)
        self.current_orientation = orientation
        
        # Konum önemli ölçüde değiştiyse yeni topolojik node oluştur
        nearest_id, distance = self.hybrid_map.find_nearest_node(x, y)
        if distance > 1.0:
            new_node_id = self.hybrid_map.add_node(x, y)
            if nearest_id is not None:
                self.hybrid_map.connect_nodes(nearest_id, new_node_id)
    
    def update_sensor_data(self, sensor_readings):
        # Haritayı yeni sensör okumaları ile güncelle
        self.obstacle_detector.update_readings(sensor_readings)
        sensor_data = self.obstacle_detector.get_sensor_data()
        
        # Sensör verilerine göre occupancy grid güncelle
        self.hybrid_map.update_grid(self.current_position, sensor_data)
        
        # Hareketli engelleri kontrol et
        moving_obstacles = self.obstacle_detector.detect_moving_obstacles()
        
        # Hareketli engel varsa, hücrelerini daha yüksek olasılıkla dolu işaretle
        for angle, distance, variance in moving_obstacles:
            if distance is not None:
                x = self.current_position[0] + distance * math.cos(angle)
                y = self.current_position[1] + distance * math.sin(angle)
                # Hareketli engeller için daha yüksek kesinlik
                self.hybrid_map.occupancy_grid.update_cell(x, y, occupied=True, sensor_accuracy=0.95)
    
    def plan_path(self):
        # Hedefe A* algoritması kullanarak yol planla veya yeniden planla
        if self.goal_position is None:
            return False
        
        # Mevcut konum ve hedefe en yakın nodeleri bul
        start_nearest_id, _ = self.hybrid_map.find_nearest_node(*self.current_position)
        goal_nearest_id, _ = self.hybrid_map.find_nearest_node(*self.goal_position)
        
        if start_nearest_id is None or goal_nearest_id is None:
            return False
        
        # Greedy algoritması ile yol bul
        self.current_path = self.path_planner.find_path(start_nearest_id, goal_nearest_id)
        return self.current_path is not None
    
    def get_next_waypoint(self):
        # Mevcut yoldaki bir sonraki hedef noktayı al
        if not self.current_path or len(self.current_path) <= 1:
            return None
        
        next_node_id = self.current_path[1]  # Mevcut nodeü atla
        next_node = self.hybrid_map.nodes[next_node_id]
        return (next_node.x, next_node.y)
    
    def check_path_validity(self):
        # Mevcut yolun hala geçerli olup olmadığını kontrol et (engel yok)
        if not self.current_path:
            return False
        
        # Yolun herhangi bir segmentinin engellenmişse kontrol et
        for i in range(len(self.current_path) - 1):
            node1 = self.hybrid_map.nodes[self.current_path[i]]
            node2 = self.hybrid_map.nodes[self.current_path[i + 1]]
            
            # Segment boyunca noktaları kontrol et
            segment_length = math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
            steps = int(segment_length / 0.1) + 1  # Her 10cm'de bir kontrol et
            
            for j in range(steps):
                ratio = j / steps
                x = node1.x + ratio * (node2.x - node1.x)
                y = node1.y + ratio * (node2.y - node1.y)
                
                # Eğer nokta doluysa, yol geçersiz
                if self.hybrid_map.occupancy_grid.is_cell_occupied(x, y):
                    return False
        
        return True
    
    def is_goal_reached(self):
        # Hedefe ulaşılıp ulaşılmadığını kontrol et
        if self.goal_position is None:
            return False
        
        # Hedefe olan mesafeyi hesapla
        distance = math.sqrt((self.current_position[0] - self.goal_position[0])**2 + 
                          (self.current_position[1] - self.goal_position[1])**2)
        
        # Hedefe mesafe eşikten küçükse ulaşıldı demektir
        return distance < self.waypoint_threshold * 1.2  # Hedef için biraz daha tolerans
    
    def calculate_movement_commands(self):
        # Hareket komutlarını hesapla (lineer ve açısal hız)
        # (lineer_hiz, açısal_hız) şeklinde döndürür
        
        # Başlangıç kontrolleri
        if self.is_goal_reached() or not self.current_path:
            return (0.0, 0.0)  # Hedefe ulaşıldığında veya yol yoksa dur
        
        next_waypoint = self.get_next_waypoint()
        if not next_waypoint:
            return (0.0, 0.0)  # Waypoint yoksa dur
        
        # Bir sonraki waypoint'e yönelim hesapla
        dx = next_waypoint[0] - self.current_position[0]
        dy = next_waypoint[1] - self.current_position[1]
        target_heading = math.atan2(dy, dx)
        
        # Yönelim hatası - ne kadar dönmemiz gerektiği
        heading_error = target_heading - self.current_orientation
        
        # Açıyı [-pi, pi] aralığına normalize et
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Waypoint'e olan mesafe
        distance = math.sqrt(dx**2 + dy**2)
        
        # Dönüş için orantısal kontrol
        # Keskin dönüşlerde daha yavaş hareket etmek için
        angular_velocity = 0.5 * heading_error
        
        # Keskin dönüşlerde ileri hızı azalt
        # Bu, hedefi geçmeyi önler
        linear_velocity = 0.3 * max(0, 1 - abs(heading_error / math.pi))
        
        # Engel algılama ve kaçınma
        # Önümüzde engel var mı kontrol et
        for i, (angle, distance) in enumerate(self.obstacle_detector.get_sensor_data()):
            if distance is None:  # Bu açıda okuma yok
                continue
                
            # Robotun mevcut yönelimine göre nispi açı
            relative_angle = angle - self.current_orientation
            
            # [-pi, pi] aralığına normalize et
            while relative_angle > math.pi:
                relative_angle -= 2 * math.pi
            while relative_angle < -math.pi:
                relative_angle += 2 * math.pi
            
            # Engel önde ve yakınsa, kaçınma hareketi yap
            if abs(relative_angle) < 0.5 and distance < self.obstacle_threshold:
                # Engelden kaçınma - dur ve dön
                linear_velocity = 0.0
                
                # Engelden uzağa dön
                if relative_angle >= 0:
                    angular_velocity = 0.5  # Sola dön
                else:
                    angular_velocity = -0.5  # Sağa dön
                break
        
        return (linear_velocity, angular_velocity)
    
    def navigate_to_goal(self):
        # Ana navigasyon döngüsü
        if not self.goal_position:
            return "Hedef ayarlanmadı"
        
        # Yol planlama veya yeniden planlama gerekiyor mu?
        if not self.current_path or not self.check_path_validity():
            success = self.plan_path()
            if not success:
                # Yol bulunamadı ama vazgeçme
                # Belki daha yaklaşır ve tekrar deneriz
                return "Yol planlanamadı - yakında tekrar denenecek"
        
        if self.is_goal_reached():
            return "Hedefe ulaşıldı"
        
        # Hareket komutlarını hesapla
        linear_vel, angular_vel = self.calculate_movement_commands()
        
        # Eğer bir waypoint'e ulaştıysak, bir sonrakine geç
        next_waypoint = self.get_next_waypoint()
        if next_waypoint:
            distance = math.sqrt((self.current_position[0] - next_waypoint[0])**2 + 
                               (self.current_position[1] - next_waypoint[1])**2)
            if distance < self.waypoint_threshold and len(self.current_path) > 2:
                self.current_path.pop(0)  # İlk waypoint'i kaldır
        
        # Hareket bilgisiyle durum döndür
        return f"Hareket ediyor: lineer={linear_vel:.2f}, açısal={angular_vel:.2f}"