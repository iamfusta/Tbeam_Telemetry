# Tbeam_Telemetry
Telemetry System with LILYGO® T-Beam V1.2 ESP32 LoRa SX1262 433 mHz

# GPS ve GNSS Sensör Entegrasyon Projesi

Bu proje, GNSS modülü (GPS, Galileo, GLONASS ve SBAS) ve MPU6050 ivmeölçer/jiroskop sensörü kullanılarak hassas konum ve hareket verilerini kaydetmeyi amaçlar. GNSS sensörleri, belirli konfigürasyonlarla çalışacak şekilde güncellenmiştir ve veriler SD karta kaydedilmektedir.

## Projenin Özellikleri
- GNSS modülü, GPS, Galileo, GLONASS ve SBAS uydu sistemleriyle çalışacak şekilde konfigüre edilmiştir.
- MPU6050 sensörü ile 3 eksenli ivmeölçer ve jiroskop verileri elde edilir.
- Veriler SD karta yazılarak kaydedilir ve bu veriler aynı zamanda seri monitöre yazdırılır.
- Uydu sinyalleri ve HDOP (İzleme Doğruluğu) değerleri kullanılarak hassas konum bilgileri elde edilir.
- Soğuk başlatma süreleri ve uydu doğruluk seviyeleri kaydedilir.

## Projenin Yapılandırılması

### Yeni Yapılandırma:
- GPS + Galileo + GLONASS + SBAS
  - GPS: Amerika Birleşik Devletleri tarafından işletilen küresel konumlama sistemi.
  - Galileo: Avrupa Birliği tarafından işletilen küresel uydu navigasyon sistemi.
  - GLONASS: Rusya tarafından işletilen küresel navigasyon uydu sistemi.
  - SBAS (EGNOS): Avrupa Geostationary Navigation Overlay Service, doğruluk artırma sistemi.

### Eski Yapılandırma:
- GPS + GLONASS + QZSS
  - QZSS: Japonya'nın bölgesel uydu sistemidir.

Yeni yapılandırmada uydu sayısının arttığı ve HDOP değerlerinin daha iyi olduğu gözlemlendi. Örneğin, soğuk başlatma sonrası 10 uydu bağlantısı ve 0.95 HDOP ile çok iyi doğruluk seviyesine ulaşıldı.

## Güncellemeler ve Değişiklikler
- GNSS modülü, 10 Hz yerine 2 Hz'de güncellenerek gereksiz GPS işlemeleri azaltıldı, bu şekilde verimlilik arttırıldı.
- Ancak bu düşük frekans, MPU6050 sensöründen elde edilen verilerin hassasiyetini olumsuz etkileyebilir.
- GNSS çalışma frekansını 5 Hz'e çıkartma denemesi yapılması önerilir.
- Cold start süreleri 3 dakika olarak kaydedildi ve bu süre içerisinde almanak verilerinin kaydedilip kaydedilmediği kontrol edilmelidir.

## Kod Parçacıkları ve Kullanılan Kütüphaneler
Bu projede kullanılan temel kütüphaneler ve başlıca kod bileşenleri şunlardır:
- **TinyGPS++** ve **SparkFun Ublox Arduino Library**: GNSS modülü için konum verilerini okuma ve işleme.
- **Adafruit_MPU6050**: MPU6050 ivmeölçer ve jiroskop verilerini okuma.
- **SD Kütüphanesi**: Verileri SD karta kaydetmek için kullanılır.

### Donanım Yapılandırması
- **ESP32 Seri Haberleşme**: GNSS modülü ve MPU6050, ESP32 kartına bağlı.
- **HSPI İle SD Kart**: Verilerin kaydedilmesi için HSPI kullanılarak SD kart arabirimi kullanılmaktadır.

### Değişkenler ve Yapılar
- **Signal yapısı**: GNSS ve sensör verilerini içeren yapı.
  - Enlem, boylam, yükseklik, hız, uydu sayısı, HDOP, zaman damgası, ivme ve gyro verileri gibi bilgiler içerir.

## Yapılandırma ve Kullanım
1. GPS modülü, MPU6050 sensörü ve SD kart modülü, ESP32 üzerinde uygun pinlere bağlanmalıdır.
2. Yazılım yüklendikten sonra seri monitör 115200 baud hızında çalıştırılmalı ve sistemin başlatılması beklenmelidir.
3. Veriler SD karta **gpsdata.txt** adlı dosyaya kaydedilir ve GNSS ve MPU6050 sensör verileri şu başlıklar altında toplanır:
   - Enlem, Boylam, Yükseklik, Hız, Uydu Sayısı, HDOP, Zaman Damgası, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ

## Gereksinimler
- **Donanım**: ESP32, GNSS Modülü, MPU6050 Sensörü, SD Kart Modülü
- **Yazılım**: Arduino IDE, ESP32 kart kütüphaneleri, GNSS ve MPU6050 kütüphaneleri

## Katkıda Bulunma
Katkıda bulunmak isteyenler için projeyi fork'layıp çekme isteği (pull request) oluşturabilirsiniz. Hataları veya geliştirme önerilerini issue oluşturarak bildirmeniz de memnuniyetle karşılanacaktır.

## Lisans
Bu proje MIT lisansı altında lisanslanmıştır. Ayrıntılar için LICENSE dosyasına bakabilirsiniz.


