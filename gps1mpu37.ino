#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>
#include "BluetoothSerial.h"  // Bluetooth Serial kütüphanesi

#define TXD_PIN 12
#define RXD_PIN 34

// HSPI pin definitions
#define HSPI_SCLK 14
#define HSPI_MISO 4
#define HSPI_MOSI 13
#define HSPI_CS 2

// Buffer büyüklüğü ve SD kart pin tanımı
#define BUFFER_SIZE 2048
String buffer = "";  // SD karta yazma için buffer kullanımı

// GPS için ikinci seri port tanımlanması
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;  // GPS verilerini işlemek için TinyGPS++ kütüphanesi kullanılıyor
SFE_UBLOX_GPS myGPS; // SparkFun UBlox GPS kütüphanesi
Adafruit_MPU6050 mpu;  // MPU6050 sensörü
BluetoothSerial SerialBT;  // Bluetooth serial nesnesi

unsigned long lastGpsRead = 0; // Son GPS okuma zamanı
unsigned long lastMpuRead = 0; // Son MPU6050 okuma zamanı
const unsigned long gpsInterval = 500; // GPS verisi için 500 ms (saniyede 2 kez) // GPS verisi için 500 ms (saniyede 2 kez)
const unsigned long mpuInterval = 27;  // MPU6050 verisi için 27 ms (saniyede yaklaşık 37 kez)

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
    while (!Serial);

    // Bluetooth Serial başlatma
    SerialBT.begin("ESP32test"); // Bluetooth cihaz adı, isterseniz değiştirebilirsiniz
    Serial.println("Bluetooth cihazı başlatıldı. Bağlantı bekleniyor...");

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

    // GPS modülünü başlatma kontrolü
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
    unsigned long currentMillis = millis();

    // GPS verilerini belirli aralıklarla okuma
    if (currentMillis - lastGpsRead >= gpsInterval / 2) {
        lastGpsRead = currentMillis;

        // GPS verilerini okuma ve çözme
        while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());

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

            // GPS verilerini yazdırma
            char gpsOutput[150];
            snprintf(gpsOutput, sizeof(gpsOutput), "%s,%s,%s,%s,%s,%s,%s", 
                     konum.lat, konum.lng, konum.altitude, konum.speed, konum.satellites, 
                     konum.hdop, konum.timestamp);
            Serial.println(gpsOutput);
            SerialBT.println(gpsOutput);

            // Buffer'a GPS verilerini ekleme
            buffer += String(gpsOutput) + "\n";
        }
    }

    // MPU6050 verilerini belirli aralıklarla okuma
    if (currentMillis - lastMpuRead >= mpuInterval) {
        lastMpuRead = currentMillis;

        // MPU6050 sensöründen ivme ve gyro verilerini okuma
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        konum.accelX = a.acceleration.x;
        konum.accelY = a.acceleration.y;
        konum.accelZ = a.acceleration.z;
        konum.gyroX = g.gyro.x;
        konum.gyroY = g.gyro.y;
        konum.gyroZ = g.gyro.z;

        // MPU6050 verilerini yazdırma
        char mpuOutput[150];
        snprintf(mpuOutput, sizeof(mpuOutput), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
                 konum.accelX, konum.accelY, konum.accelZ, 
                 konum.gyroX, konum.gyroY, konum.gyroZ);
        Serial.println(mpuOutput);
        SerialBT.println(mpuOutput);

        // Buffer'a MPU6050 verilerini ekleme
        buffer += String(mpuOutput) + "\n";
    }

    // Buffer dolduğunda veya belirli bir süre geçtiğinde dosyaya yaz
    if (buffer.length() >= BUFFER_SIZE) {
        writeBufferToSD();
    }
}

void writeBufferToSD() {
    if (sdCardInitialized) {
        dataFile = SD.open(currentFileName, FILE_APPEND); // Dosya ekleme modu
        if (dataFile) {
            dataFile.print(buffer);  // Buffer'daki veriyi dosyaya yaz
            dataFile.flush();        // Dosyaya yazmayı tamamlayıp tamponu temizle
                        buffer = ""; // Buffer'ı temizle
            dataFile.close();
        } else {
            Serial.println("Veri dosyasını açma hatası!");
            SerialBT.println("Veri dosyasını açma hatası!");
            sdCardInitialized = false; // Hata mesajını yalnızca bir kere yazdırmak için kontrol bayrağını ayarlayın
        }
    }
}

void loopEnd() {
    // Bu fonksiyon SD kartı güvenli bir şekilde kapatmak için kullanılır, 
    // eğer dosya açık kalmışsa onu kapatır.
    if (dataFile) {
        dataFile.close();
    }
}
