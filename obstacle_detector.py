import math
import numpy as np

# Engel algılama sistemi - Ultrasonik/IR sensörleri kullanarak 
# Kim ve Do [11] referansındaki hareketli engel algılama konseptini uygular
class ObstacleDetector:
    def __init__(self, sensor_sayisi=8):
        self.num_sensors = sensor_sayisi
        # Sensör açıları (radyan) - robotun çevresine eşit aralıklarla yerleştirilmiş
        self.sensor_angles = [2 * math.pi * i / sensor_sayisi for i in range(sensor_sayisi)]
        self.previous_readings = [None] * sensor_sayisi  # Önceki sensör okumaları
        self.current_readings = [None] * sensor_sayisi   # Mevcut sensör okumaları
        
        # Hareketli engel algılama için (Kim ve Do [11])
        self.reading_history = [[] for _ in range(sensor_sayisi)]  # Son okumaları sakla
        self.history_size = 5  # Her sensör için saklanacak okuma sayısı
    
    def update_readings(self, new_readings):
        # Sensör okumalarını güncelle ve hareketli engel tespiti için değişimleri takip et
        if len(new_readings) != self.num_sensors:
            raise ValueError("Okuma sayısı sensör sayısıyla eşleşmeli")
        
        # Okumaları güncelle
        self.previous_readings = self.current_readings.copy()
        self.current_readings = new_readings
        
        # Her sensör için geçmişi güncelle
        for i in range(self.num_sensors):
            self.reading_history[i].append(new_readings[i])
            # Geçmiş boyutu sınırlı tut
            if len(self.reading_history[i]) > self.history_size:
                self.reading_history[i].pop(0)
    
    def detect_moving_obstacles(self, threshold=0.2):
        # Sensör okumalarındaki zamansal değişimleri analiz ederek hareketli engelleri tespit et
        # Kim ve Do [11]'den esinlenilmiştir
        moving_obstacles = []
        
        for i in range(self.num_sensors):
            history = self.reading_history[i]
            if len(history) < 2:
                continue  # Hareket tespiti için en az 2 okuma gerekli
            
            # None okumaları atla (engel tespit edilmedi)
            if None in history:
                continue
                
            # Hareketi tespit etmek için okumalardaki varyansı hesapla
            variance = np.var(history)  # Yüksek varyans hareket olduğunu gösterir
            
            if variance > threshold:
                # Hareket tespit edildi - açı, mesafe ve varyansı kaydet
                moving_obstacles.append((self.sensor_angles[i], history[-1], variance))
        
        return moving_obstacles
    
    def get_sensor_data(self):
        # Mevcut sensör okumalarını açılarıyla birlikte döndür
        return [(self.sensor_angles[i], self.current_readings[i]) for i in range(self.num_sensors)]
        
    def visualize_obstacles(self):
        # Tespit edilen engellerin görselleştirilmesi için (ileride eklenebilir)
        pass