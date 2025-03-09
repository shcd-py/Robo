# Yangın Söndürme Robotu: Navigasyon ve Engel Kaçınma Modülü

## Genel Bakış
Bu modül, Yangın Söndürme Robotu bitirme projesi için kapsamlı bir navigasyon ve engel kaçınma sistemi sağlar. Sistem aşağıdaki özellikleri sunar:

1. Ortam tespiti ve haritalama
2. Hem statik hem de hareketli engelleri tespit etme
3. Greedy Best-First Search algoritması kullanarak verimli yol planlama
4. Tespit edilen yangına güvenli bir şekilde navigasyon
5. Engeller algılandığında yolları dinamik olarak yeniden planlama

Uygulama, bitirme projesi dokümantasyonundaki [11] - [17] arası referanslara dayanmakta ve robotik alanında yerleşik algoritmaları ve teknikleri kullanmaktadır.

## Proje Yapısı
Kod, OOP prensiplerine dayalı modüler dosyalara ayrılmıştır:

- `occupancy_grid.py`: Izgara tabanlı ortam haritalama (Moravec [12], Meyer-Delius [13])
- `topological_mapping.py`: Hibrit ızgara-topolojik haritalama (Niijima [14])
- `greedy_algorithm.py`: Greedy Best-First Search yol planlama algoritması (Goto [15], Han [16] referanslarından uyarlanmıştır)
- `obstacle_detector.py`: Hareketli nesneler dahil engel tespiti (Kim [11])
- `navigation_system.py`: Tam navigasyon için tüm bileşenleri entegre eder (Liu [17])
- `simulation.py`: Navigasyonu test etmek için simülasyon ortamı sağlar
- `main.py`: Simülasyonu çalıştırmak için giriş noktası

## Gereksinimler
- Python 3.6+
- NumPy

## Kullanım
Simülasyonu çalıştırmak için:

```bash
python main.py
```

## Algoritma Detayları

### Occupancy Grid Haritalama
Moravec ve Elfes [12]'e dayalı ve Meyer-Delius [13]'den dinamik ortam işleme ile geliştirilmiştir. Bu yaklaşım, ortamı her hücrenin dolu olma olasılığına sahip olduğu bir ızgara olarak temsil eder.

### Hibrit Haritalama
Niijima [14]'dan gelen yaklaşımı uygular, detaylı ızgara haritalarını daha verimli navigasyon planlaması için topolojik temsillerle birleştirir.

### Greedy Best-First Search Yol Planlama
Goto [15] ve Han [16] tarafından açıklanan tekniklerden uyarlanmıştır. A* algoritmasından farklı olarak yalnızca hedefe olan uzaklığı (sezgisel) dikkate alır, bu da yangın konumu bilindiğinde daha hızlı ve daha verimli navigasyon sağlar.

### Hareketli Engel Tespiti
Kim ve Do [11]'ya dayalı olarak, sensör okumalarındaki değişiklikleri zaman içinde analiz ederek hareketli engelleri tespit eder ve izler.

### Yapılandırılmamış Ortamlarda Navigasyon
Sistem, Liu [17]'den hibrit yol planlama yaklaşımını uygular, karmaşık, yapılandırılmamış ortamlarda navigasyon için uygundur.

## Yangın Algılama ile Entegrasyon
Bu navigasyon sistemi, yangın algılama modülü ile birlikte çalışacak şekilde tasarlanmıştır. Yangın tespit edildiğinde, koordinatlar navigasyon sistemine iletilir, sistem bir yol planlar ve robotu yangın konumuna doğru yönlendirir.

## Referanslar
[11] Kim ve Do - Tek kamera kullanan hareketli engel algılama  
[12] Moravec ve Elfes - Occupancy Grid Haritalama  
[13] Meyer-Delius vd. - Dinamik Ortam İşleme  
[14] Niijima vd. - Hibrit Grid-Topolojik Haritalar  
[15] Goto vd. - Yol planlama algoritmaları  
[16] Han vd. - Mobil Robotlar için Navigasyon Algoritmaları  
[17] Liu vd. - Yapılandırılmamış Ortamlar için Hibrit Yol Planlama