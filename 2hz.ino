/*
txt dosya adı fixed.
100 saatten fazla test edildi.
GPS 10 Hz komutu çıkartıldı.
GPS güncellemesi 2 Hz ile değiştirildi.
---> Bunun kötü yanı, MPU sensörünün verilerinin kalitesinin düşmesi oldu.
---> İyi yanı ise, GPS'in boş işlemleri azaltıldı.

MPU sensörünün çalışma frekansı yüksek kalmalı.
GPS çalışma frekansı 2 Hz olmalı.
Mümkünse GNSS çalışma frekansı 5 Hz olacak şekilde bir deneme daha yapılmalı.
Oluşturulan dosya isimleri benzersiz hale getirilmeli.
Sensörlerin fazla akım çektiğini düşündüğüm için bazı sorunlar yaşandı ve bu yüzden harici 5V ile beslemeleri sağladım.

UPDATE: Cold start 3 dakika sürüyor. Almanak verilerinin kaydedilip kaydedilmediğini kontrol etmemiz gerek.
Cold start sonrası 10 uydu ile, HDOP 0.95 seviyesinde konum almaya başladık.

Yeni konfigürasyon: GPS + Galileo + GLONASS + SBAS
GPS: Amerika Birleşik Devletleri tarafından işletilen küresel konumlama sistemi.
Galileo: Avrupa Birliği tarafından işletilen küresel uydu navigasyon sistemi.
GLONASS: Rusya tarafından işletilen küresel navigasyon uydu sistemi.
SBAS (EGNOS): Avrupa Geostationary Navigation Overlay Service, doğruluk artırma sistemi.

Eski konfigürasyon: GPS + GLONASS + QZSS
GPS (Global Positioning System)
GLONASS (GLObal NAvigation Satellite System)
QZSS (Quasi-Zenith Satellite System)

Eski kodda 6 uydu ile, HDOP 1.2 seviyesinde bu çok iyi bir doğruluk seviyesi. Ölçümlerin evde cam kenarında yapıldığı düşünülürse oldukça iyi.
Şu an:
- 10 uydu, HDOP 0.95: Mükemmel doğruluk seviyesi
- 9 uydu, HDOP 1.15: Çok iyi doğruluk seviyesi
- 8 uydu, HDOP 1.40-1.50: Çok iyi doğruluk seviyesi
- 8 uydu, HDOP 1.50-1.60: İyi doğruluk seviyesi
- 6 uydu, HDOP 1.60-1.70: İyi doğruluk seviyesi
*/


#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>

#define TXD_PIN 12
#define RXD_PIN 34

// HSPI pin definitions
#define HSPI_SCLK 14
#define HSPI_MISO 4
#define HSPI_MOSI 13
#define HSPI_CS 2

// GPS için ikinci seri port tanımlanması
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;  // GPS verilerini işlemek için TinyGPS++ kütüphanesi kullanılıyor
SFE_UBLOX_GPS myGPS; // SparkFun UBlox GPS kütüphanesi
Adafruit_MPU6050 mpu;  // MPU6050 sensörü

unsigned long lastSatelliteHdopRead = 0; // Son uydu ve HDOP okuma zamanı
int satellites = 0; // Uydu sayısı
int hdop = 0; // HDOP değeri
bool sdCardInitialized = false;  // SD kartın başlatılıp başlatılmadığını kontrol eder
File dataFile; // SD kart üzerindeki dosya nesnesi
String currentFileName = "/gpsdata.txt"; // Oluşturulan dosyanın adı

// GNSS verilerini tutmak için yapı tanımı
struct Signal {
    char lat[10];        // Enlem bilgisi
    char lng[10];        // Boylam bilgisi
    char altitude[10];   // Yükseklik bilgisi
    char speed[10];      // Hız bilgisi
    char satellites[4];  // Uydu sayısı bilgisi
    char hdop[5];        // HDOP değeri
    char timestamp[20];  // Zaman damgası (T ve Z karakterleri olmadan)
    float accelX, accelY, accelZ; // İvme verileri (X, Y, Z)
    float gyroX, gyroY, gyroZ;    // Gyro verileri (X, Y, Z)
};

Signal konum;  // Konum verilerini tutmak için yapı oluşturma

void setup() {
    // Seri haberleşmeyi başlat (PC ile iletişim)
    Serial.begin(115200);
    while (!Serial) {
        ; // Seri portun bağlanmasını bekleyin
    }

    // SPI arayüzünü başlat (SD kart için)
    SPI.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
    pinMode(HSPI_CS, OUTPUT);

    // SD kartı başlatma
    Serial.print("SD kart başlatılıyor...");
    if (!SD.begin(HSPI_CS)) {
        Serial.println("SD kart başlatma başarısız oldu veya kart takılı değil.");
        sdCardInitialized = false;
    } else {
        Serial.println("SD kart başarıyla başlatıldı.");
        sdCardInitialized = true;

        // SD kartta dosya oluşturma veya mevcut dosyayı belirleme
        int fileIndex = 0;
        while (SD.exists(currentFileName)) {
            fileIndex++;
            currentFileName = "/gpsdata" + String(fileIndex) + ".txt";
        }

        // Yeni dosyayı oluşturma
        dataFile = SD.open(currentFileName, FILE_WRITE);
        if (dataFile) {
            dataFile.println("Lat,Lng,Altitude,Speed,Satellites,HDOP,Timestamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ"); // Başlık satırı
            dataFile.close();
        } else {
            Serial.println("Dosya oluşturulamadı!");
            sdCardInitialized = false;
        }
    }

    // GPS modülünün seri haberleşmesini başlatma
    gpsSerial.begin(9600, SERIAL_8N1, RXD_PIN, TXD_PIN);
    Wire.begin();  // MPU6050 için I2C haberleşme başlatma

    Serial.println("Starting setup...");

    // GPS modülü başlatma kontrolü
    if (myGPS.begin(gpsSerial)) {
        Serial.println("GPS initialized successfully.");
    } else {
        Serial.println("GPS initialization failed.");
    }

    // GPS modülünün baud hızını artırma
    myGPS.setSerialRate(115200);
    gpsSerial.updateBaudRate(115200);
    Serial.println("Baud rate set to 115200.");

    // MPU6050 sensörünü başlatma
    if (!mpu.begin()) {
        Serial.println("MPU6050 initialization failed!");
        while (1); // Başarısız olursa sonsuza kadar bekler
    }
    Serial.println("MPU6050 initialized successfully.");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("Setup complete.");
}

void loop() {
    // GPS verilerini okuma ve çözme
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    // GPS konumu güncellenmişse verileri işleme
    if (gps.location.isUpdated()) {
        // GPS verilerini al
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        float altitude = gps.altitude.meters();
        float speed = gps.speed.kmph();
        int satellites = gps.satellites.value();
        float hdop = gps.hdop.hdop();

        // GPS verilerini konum yapısına yazma
        snprintf(konum.lat, 10, "%f", latitude); 
        snprintf(konum.lng, 10, "%f", longitude); 
        snprintf(konum.altitude, 10, "%f", altitude); 
        snprintf(konum.speed, 10, "%f", speed); 
        snprintf(konum.satellites, 4, "%d", satellites); 
        snprintf(konum.hdop, 5, "%f", hdop); 
        snprintf(konum.timestamp, 20, "%04d-%02d-%02d %02d:%02d:%02d", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());

        // MPU6050 sensöründen ivme ve gyro verilerini okuma
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        konum.accelX = a.acceleration.x;
        konum.accelY = a.acceleration.y;
        konum.accelZ = a.acceleration.z;
        konum.gyroX = g.gyro.x;
        konum.gyroY = g.gyro.y;
        konum.gyroZ = g.gyro.z;

        // Verileri seri monitöre yazdırma
        Serial.println(String(konum.lat) + "," + String(konum.lng) + "," + String(konum.altitude) + "," + String(konum.speed) + "," + String(konum.satellites) + "," + String(konum.hdop) + "," + String(konum.timestamp) + "," + String(konum.accelX) + "," + String(konum.accelY) + "," + String(konum.accelZ) + "," + String(konum.gyroX) + "," + String(konum.gyroY) + "," + String(konum.gyroZ));

        // Eğer SD kart başlatıldıysa verileri SD karta yazma
        if (sdCardInitialized) {
            dataFile = SD.open(currentFileName, FILE_APPEND); // Dosya ekleme modu
            if (dataFile) {
                dataFile.println(String(konum.lat) + "," + String(konum.lng) + "," + String(konum.altitude) + "," + String(konum.speed) + "," + String(konum.satellites) + "," + String(konum.hdop) + "," + String(konum.timestamp) + "," + String(konum.accelX) + "," + String(konum.accelY) + "," + String(konum.accelZ) + "," + String(konum.gyroX) + "," + String(konum.gyroY) + "," + String(konum.gyroZ)); 
                dataFile.close();
            } else {
                Serial.println("Veri dosyasını açma hatası!");
            }
        }
    }
}

void setFileName(String fileName) {
    currentFileName = fileName;
    // Yeni dosyayı başlık satırıyla oluşturma
    if (sdCardInitialized) {
        dataFile = SD.open(currentFileName, FILE_WRITE);
        if (dataFile) {
            dataFile.println("Lat,Lng,Altitude,Speed,Satellites,HDOP,Timestamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ"); // Başlık satırı
            dataFile.close();
        } else {
            Serial.println("Yeni dosya oluşturulamadı!");
        }
    }
}
