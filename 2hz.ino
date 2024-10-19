/*
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

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
SFE_UBLOX_GPS myGPS;
Adafruit_MPU6050 mpu;

unsigned long lastSatelliteHdopRead = 0; // Son uydu ve HDOP okuma zamanı
int satellites = 0; // Uydu sayısı
int hdop = 0; // HDOP değeri
bool sdCardInitialized = false;
File dataFile; // Dosya nesnesi

struct Signal {
    char lat[10];        // Enlem
    char lng[10];        // Boylam
    char altitude[10];   // Yükseklik
    char speed[10];      // Hız
    char satellites[4];  // Uydu sayısı
    char hdop[5];        // HDOP
    char timestamp[20];  // Zaman damgası (T ve Z karakterleri olmadan)
    float accelX, accelY, accelZ; // İvme verileri
    float gyroX, gyroY, gyroZ;    // Gyro verileri
};

Signal konum;

// GNSS yapılandırmasını genişletmek için UBX-CFG-GNSS komutunu gönder
void configureGNSS() {
    uint8_t setNav5[] = {
        0xB5, 0x62,  // Sync chars
        0x06, 0x3E,  // Class and ID
        0x24, 0x00,  // Length
        0x00,        // msgVer
        0x20,        // numTrkChHw (number of tracking channels)
        0x00,        // numTrkChUse (number of tracking channels to use)
        0x01, 0x00,  // gnssId, resTrkCh, maxTrkCh, reserved
        0x01,        // GPS enabled
        0x01,        // SBAS enabled
        0x00, 0x00,  // reserved
        0x02, 0x00,  // Galileo enabled
        0x01, 0x00,  // reserved
        0x04, 0x00,  // GLONASS enabled
        0x01, 0x00,  // reserved
        0x00, 0x00, 0x00, 0x00,  // reserved
        0x00, 0x00, 0x00, 0x00,  // reserved
        0x00, 0x00, 0x00, 0x00,  // reserved
        0x00, 0x00, 0x00, 0x00,  // reserved
        0x00, 0x00, 0x00, 0x00,  // reserved
        0x00, 0x00, 0x00, 0x00,  // reserved
        0x00, 0x00, 0x00, 0x00,  // reserved
        0x00, 0x00, 0x00, 0x00,  // reserved
        0x00, 0x00, 0x00, 0x00,  // reserved
    };

    // Checksum calculation
    uint8_t ckA = 0, ckB = 0;
    for (int i = 2; i < sizeof(setNav5) - 2; i++) {
        ckA += setNav5[i];
        ckB += ckA;
    }
    setNav5[sizeof(setNav5) - 2] = ckA;
    setNav5[sizeof(setNav5) - 1] = ckB;

    gpsSerial.write(setNav5, sizeof(setNav5));
    Serial.println("Sent UBX-CFG-GNSS command.");
}

void setup() {
    // Seri haberleşmeyi başlat
    Serial.begin(115200);
    while (!Serial) {
        ; // Seri portun bağlanmasını bekleyin
    }

    // SPI arayüzünü doğru pin isimleri ile başlat
    SPI.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
    pinMode(HSPI_CS, OUTPUT);

    // SD kartı başlat
    Serial.print("SD kart başlatılıyor...");
    if (!SD.begin(HSPI_CS)) {
        Serial.println("SD kart başlatma başarısız oldu veya kart takılı değil.");
        sdCardInitialized = false;
    } else {
        Serial.println("SD kart başarıyla başlatıldı.");
        sdCardInitialized = true;

        // Dosyayı oluşturma veya açma
        dataFile = SD.open("/gpsdata.txt", FILE_WRITE);
        if (dataFile) {
            dataFile.println("Lat,Lng,Altitude,Speed,Satellites,HDOP,Timestamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ"); // Başlık satırını yazma
            dataFile.close();
        } else {
            Serial.println("Dosya oluşturulamadı!");
            sdCardInitialized = false;
        }
    }

    // GPS modülü ve diğer sensörlerin başlatılması
    gpsSerial.begin(9600, SERIAL_8N1, RXD_PIN, TXD_PIN);
    Wire.begin();

    Serial.println("Starting setup...");

    if (myGPS.begin(gpsSerial)) {
        Serial.println("GPS initialized successfully.");
    } else {
        Serial.println("GPS initialization failed.");
    }

    // GPS modülünün baud hızını 115200'e değiştirme
    myGPS.setSerialRate(115200);
    gpsSerial.updateBaudRate(115200);
    Serial.println("Baud rate set to 115200.");



    // GNSS yapılandırmasını genişletmek için UBX-CFG-GNSS komutunu gönder
    configureGNSS();

    // MPU6050 Başlatma
    if (!mpu.begin()) {
        Serial.println("MPU6050 initialization failed!");
        while (1);
    }
    Serial.println("MPU6050 initialized successfully.");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("Setup complete.");
}

void loop() {
    // GPS verilerini okuma ve işleme
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        float altitude = gps.altitude.meters();
        float speed = gps.speed.kmph();
        int satellites = gps.satellites.value();
        float hdop = gps.hdop.hdop();

        snprintf(konum.lat, 10, "%f", latitude); snprintf(konum.lng, 10, "%f", longitude); snprintf(konum.altitude, 10, "%f", altitude); snprintf(konum.speed, 10, "%f", speed); snprintf(konum.satellites, 4, "%d", satellites); snprintf(konum.hdop, 5, "%f", hdop); snprintf(konum.timestamp, 20, "%04d-%02d-%02d %02d:%02d:%02d", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());


        // MPU6050 sensöründen veri okuma
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

        // Eğer SD kart başlatıldıysa, verileri SD karta yaz
        if (sdCardInitialized) {
            dataFile = SD.open("/gpsdata.txt", FILE_APPEND); // Dosya ekleme modu
            if (dataFile) {
                dataFile.println(String(konum.lat) + "," + String(konum.lng) + "," + String(konum.altitude) + "," + String(konum.speed) + "," + String(konum.satellites) + "," + String(konum.hdop) + "," + String(konum.timestamp) + "," + String(konum.accelX) + "," + String(konum.accelY) + "," + String(konum.accelZ) + "," + String(konum.gyroX) + "," + String(konum.gyroY) + "," + String(konum.gyroZ)); 
                dataFile.close();
            } else {
                Serial.println("Veri dosyasını açma hatası!");
            }
        }
    }
}
