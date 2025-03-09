import math

# Greedy Best-First Search algoritması
# Ateşin konumunu bildiğimiz için hedefe direkt yönelme yaklaşımı daha uygun
class GreedyBestFirst:
    def __init__(self, hybrid_map):
        self.map = hybrid_map
    
    # Düz çizgi mesafesi hesaplama (Öklid)
    def heuristic(self, node_id, goal_id):
        node = self.map.nodes[node_id]
        goal = self.map.nodes[goal_id]
        return math.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2)
    
    # Greedy Best-First Search kullanarak yol bulma
    def find_path(self, start_id, goal_id):
        # Hata kontrolü
        if start_id not in self.map.nodes or goal_id not in self.map.nodes:
            print("Başlangıç veya hedef düğüm geçersiz")
            return None
        
        # Veri yapılarını başlat
        open_set = {start_id}  # Keşfedilecek düğümler
        closed_set = set()     # Keşfedilmiş düğümler
        came_from = {}         # Geri izleme için
        
        # Greedy için sadece h değerini (hedefe uzaklık) tutuyoruz
        h_score = {start_id: self.heuristic(start_id, goal_id)}
        
        # Ana döngü
        while open_set:
            # En düşük h değerine sahip düğümü bul
            current_id = min(open_set, key=lambda id: h_score.get(id, float('inf')))
            
            # Hedefe ulaştık mı?
            if current_id == goal_id:
                # Yolu oluştur
                path = []
                while current_id in came_from:
                    path.append(current_id)
                    current_id = came_from[current_id]
                path.append(start_id)
                path.reverse()
                return path
            
            # Mevcut düğümü işlenmiş olarak işaretle
            open_set.remove(current_id)
            closed_set.add(current_id)
            
            # Tüm komşu düğümleri kontrol et
            for neighbor_id, distance in self.map.nodes[current_id].get_connections():
                # Zaten işlenmiş düğümü atla
                if neighbor_id in closed_set:
                    continue
                
                # Sadece hedefe olan uzaklığı hesapla
                neighbor_h = self.heuristic(neighbor_id, goal_id)
                
                # Yeni düğüm veya daha iyi yol?
                if neighbor_id not in open_set:
                    open_set.add(neighbor_id)
                    h_score[neighbor_id] = neighbor_h
                    came_from[neighbor_id] = current_id
                elif neighbor_h < h_score.get(neighbor_id, float('inf')):
                    h_score[neighbor_id] = neighbor_h
                    came_from[neighbor_id] = current_id
        
        # Yol bulunamadı
        print("Başlangıçtan hedefe yol bulunamadı")
        return None