import numpy as np
import math

# Occupancy Grid Mapping algoritması
# Moravec ve Elfes [12]'in yaklaşımı temel alınmıştır
# Meyer-Delius vd. [13]'in dinamik ortam değişikliği geliştirmeleri eklenmiştir
class OccupancyGrid:
    def __init__(self, width, height, resolution=0.1):
        # Grid'i bilinmeyen olasılıklarla (0.5) başlat
        self.width = width
        self.height = height
        self.resolution = resolution  # metre/hücre
        self.grid = np.ones((int(height/resolution), int(width/resolution))) * 0.5
        self.cells_x = int(width/resolution)
        self.cells_y = int(height/resolution)
        
        # Dinamik ortam güncelleme parametreleri (Meyer-Delius [13])
        self.decay_factor = 0.01  # Kesinliğin zamanla azalma oranı
    
    def update_cell(self, x, y, occupied, sensor_accuracy=0.9):
        # Sensör okuması kullanarak grid hücresini Bayes güncellemesi ile güncelle
        # Dünya koordinatlarını grid indekslerine dönüştür
        grid_x = min(max(int(x / self.resolution), 0), self.cells_x - 1)
        grid_y = min(max(int(y / self.resolution), 0), self.cells_y - 1)
        
        # Mevcut olasılık
        p_current = self.grid[grid_y, grid_x]
        
        # Bayes güncelleme kuralı
        if occupied:
            # Sensör bir engel algıladı
            p_new = (sensor_accuracy * p_current) / \
                   (sensor_accuracy * p_current + (1 - sensor_accuracy) * (1 - p_current))
        else:
            # Sensör boş alan algıladı
            p_new = ((1 - sensor_accuracy) * p_current) / \
                   ((1 - sensor_accuracy) * p_current + sensor_accuracy * (1 - p_current))
        
        self.grid[grid_y, grid_x] = p_new
    
    def update_from_sensor_data(self, robot_pos, sensor_readings):
        # Bir dizi sensör okumasından grid'i güncelle (mesafeler ve açılar)
        robot_x, robot_y = robot_pos
        
        for angle, distance in sensor_readings:
            if distance is None or distance > 4.0:  # Maksimum sensör menzili
                continue
                
            # Algılanan noktanın koordinatlarını hesapla
            x = robot_x + distance * math.cos(angle)
            y = robot_y + distance * math.sin(angle)
            
            # Işın boyunca engele kadar olan hücreleri boş olarak işaretle
            ray_length = min(distance, 4.0)  # Sensör menziline kadar sınırla
            steps = int(ray_length / (self.resolution * 0.5))
            
            for i in range(steps):
                # Işın boyunca noktayı hesapla
                ratio = i / steps
                ray_x = robot_x + ratio * ray_length * math.cos(angle)
                ray_y = robot_y + ratio * ray_length * math.sin(angle)
                
                # Boş alan olarak işaretle (dolu değil)
                if i < steps - 1:  # Son nokta hariç tüm noktalar
                    self.update_cell(ray_x, ray_y, occupied=False)
            
            # Gerçekte algılanan noktayı dolu olarak işaretle
            if distance < 4.0:  # Sadece menzil içindeyse
                self.update_cell(x, y, occupied=True)
    
    def apply_time_decay(self):
        # Dinamik ortamları dikkate almak için zaman tabanlı bozunma uygula
        # Meyer-Delius vd. [13] tarafından önerildiği gibi
        # Olasılıkları belirsizliğe (0.5) doğru hareket ettir
        self.grid = self.grid + self.decay_factor * (0.5 - self.grid)
    
    def is_cell_occupied(self, x, y, threshold=0.7):
        # Bir hücrenin olasılık eşiğine göre dolu kabul edilip edilmediğini kontrol et
        grid_x = min(max(int(x / self.resolution), 0), self.cells_x - 1)
        grid_y = min(max(int(y / self.resolution), 0), self.cells_y - 1)
        return self.grid[grid_y, grid_x] >= threshold