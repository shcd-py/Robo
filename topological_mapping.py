import math
from occupancy_grid import OccupancyGrid

# Topolojik haritalama için node sınıfı (Niijima vd. [14])
class TopologicalNode:
    def __init__(self, x, y, node_id):
        self.x = x
        self.y = y
        self.id = node_id
        self.connections = []  # Bağlantılı node ID'leri ve aralarındaki mesafeler
    
    def add_connection(self, target_id, distance):
        # Başka bir nodee bağlantı ekle
        # Bağlantı zaten varsa, tekrar eklemeyi önle
        for conn_id, _ in self.connections:
            if conn_id == target_id:
                return
        
        self.connections.append((target_id, distance))
    
    def get_connections(self):
        # Bağlantılı nodelerin listesini döndür
        return self.connections


# Niijima vd. [14] tarafından önerilen Hibrit Grid-Topolojik haritalama
# Detaylı occupancy grid'i yüksek seviyeli topolojik grafik ile birleştirir
class HybridMap:
    def __init__(self, width, height, resolution=0.1):
        self.occupancy_grid = OccupancyGrid(width, height, resolution)
        self.nodes = {}  # Topolojik nodeler
        self.next_node_id = 0
    
    def add_node(self, x, y):
        # Verilen koordinatlarda yeni bir topolojik node ekle
        node = TopologicalNode(x, y, self.next_node_id)
        self.nodes[self.next_node_id] = node
        
        # Yakında olan mevcut nodelerle bağlantı kurmaya çalış
        # Bu, grafın daha bağlantılı olmasını ve daha fazla yol seçeneği olmasını sağlar
        for node_id, other_node in self.nodes.items():
            if node_id == self.next_node_id:
                continue  # Kendisiyle bağlantı kurma
                
            # nodeler arasındaki mesafeyi hesapla
            dist = math.sqrt((node.x - other_node.x)**2 + (node.y - other_node.y)**2)
            
            # Eşik değeri içindeyse bağla (uyarlanabilir olabilir)
            if dist < 2.0:  # 2 metre içindeki nodeleri bağla
                node.add_connection(node_id, dist)
                other_node.add_connection(self.next_node_id, dist)
        
        # node sayacını artır ve ID'yi döndür
        current_id = self.next_node_id
        self.next_node_id += 1
        return current_id
    
    def connect_nodes(self, node1_id, node2_id):
        # Topolojik grafikteki iki nodeü birbirine bağla
        if node1_id not in self.nodes or node2_id not in self.nodes:
            return False
        
        node1 = self.nodes[node1_id]
        node2 = self.nodes[node2_id]
        
        # nodeler arasındaki Öklid mesafesini hesapla
        distance = math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
        
        # Çift yönlü bağlantı ekle
        node1.add_connection(node2_id, distance)
        node2.add_connection(node1_id, distance)
        return True
    
    def update_grid(self, robot_pos, sensor_readings):
        # Sensör okumalarına göre occupancy grid'i güncelle
        self.occupancy_grid.update_from_sensor_data(robot_pos, sensor_readings)
        self.occupancy_grid.apply_time_decay()
    
    def find_nearest_node(self, x, y):
        # Verilen koordinatlara en yakın topolojik nodeü bul
        nearest_id = None
        min_distance = float('inf')
        
        for node_id, node in self.nodes.items():
            distance = math.sqrt((node.x - x)**2 + (node.y - y)**2)
            if distance < min_distance:
                min_distance = distance
                nearest_id = node_id
        
        return nearest_id, min_distance