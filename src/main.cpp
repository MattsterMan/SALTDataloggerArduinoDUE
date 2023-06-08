#include <Arduino.h>
#include <SD.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GPS.h>

#define SEALEVELPRESSURE_HPA (1007.00)
#define DELAYTIME 100
#define CHIPSELECT 4
#define DATABUFFERSIZE 50
#define GPSSerial Serial1
#define GPSECHO true

double altitudeBuffer[DATABUFFERSIZE] = {};
double temperatureBuffer[DATABUFFERSIZE] = {};
double pressureBuffer[DATABUFFERSIZE] = {};
double humidityBuffer[DATABUFFERSIZE] = {};
int fixBuffer[DATABUFFERSIZE] = {};
float longitudeBuffer[DATABUFFERSIZE] = {};
float latitudeBuffer[DATABUFFERSIZE] = {};
float speedBuffer[DATABUFFERSIZE] = {};
float angleBuffer[DATABUFFERSIZE] = {};
float altitudeGPSBuffer[DATABUFFERSIZE] = {};

char printfBuffer[120];
const int START_BUTTON = 7;
const int END_BUTTON = 3;
uint32_t timer = millis();

Adafruit_BME280 bme; // I2C
Adafruit_GPS GPS(&GPSSerial);

void setup() {
    Serial.begin(115200);
    pinMode(START_BUTTON, INPUT_PULLUP);
    while(!Serial);
    while(digitalRead(START_BUTTON) == HIGH);    // time to get serial running and button pressed

    Serial.println("Initializing BME280...");

    unsigned status;
    // initializes the BME280
    status = bme.begin();
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        exit(0);
    }

    Serial.println("Initializing SD card...");
    // see if the card is present and can be initialized:
    if (!SD.begin(CHIPSELECT)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        exit(0);
    }
    Serial.println("Successfully Initialized");
    // add headings for data in the datalog file
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.println("BME Altitude (m), Temperature (C), Pressure (kpa), Humidity (%), Fix (bool), Longitude (N/S), Latitude (E/W), Speed (knots), Angle (degrees), GPS Altitude (m)");
        dataFile.close();
        Serial.println("Header Created");
    }
    else {
        Serial.println("error opening datalog.csv");
        exit(0);
    }

    // begin serial connection using hardware serial of GPS
    GPS.begin(9600);
    Serial.println("GPS Serial Connected");
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //sets the NMEA sentence output to RMC+GGA
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //1HZ recommended for parsing data
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    GPSSerial.println(PMTK_Q_RELEASE);

    delay(1000);
}

void writeToSD() {
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
        for (int i = 0; i < DATABUFFERSIZE; ++i) {
            sprintf(printfBuffer, "%.2f, %.2f, %.2f, %.2f, %i, %.6f, %.6f, %.2f, %.2f, %.2f",
                    altitudeBuffer[i],
                    temperatureBuffer[i],
                    pressureBuffer[i],
                    humidityBuffer[i],
                    fixBuffer[i],
                    latitudeBuffer[i],
                    longitudeBuffer[i],
                    speedBuffer[i],
                    angleBuffer[i],
                    altitudeGPSBuffer[i]);
            dataFile.println(printfBuffer);
        }
        dataFile.close();
        Serial.println("Write Successful");
    }
    else {
        Serial.println("error opening datalog.csv");
    }
}

void writeToDataBuffer() {
    GPS.parse(GPS.lastNMEA());
    int index = 0;
    for (int i = 0; i < DATABUFFERSIZE; ++i) {
        while (GPS.available()){
            char nmea = GPS.read();
        }
        if (GPS.newNMEAreceived()) {  // clears the newNMEAreceived flag
            if (!GPS.parse(GPS.lastNMEA()))
                return;
        }
        altitudeBuffer[i] = bme.readAltitude(SEALEVELPRESSURE_HPA);
        temperatureBuffer[i] = bme.readTemperature();
        pressureBuffer[i] = bme.readPressure() / 1000.0F;
        humidityBuffer[i] = bme.readHumidity();
        fixBuffer[i] = (int)GPS.fix;
        longitudeBuffer[i] = GPS.longitudeDegrees;
        latitudeBuffer[i] = GPS.latitudeDegrees;
        speedBuffer[i] = GPS.speed;
        angleBuffer[i] = GPS.angle;
        altitudeGPSBuffer[i] = GPS.altitude;
        sprintf(printfBuffer, "%d: %.2f m, %.2f C, %.2f kpa, %.2f %%, %i, %.6f, %.6f, %.2f knots, %.2f degrees, %.2f m",
                index,
                altitudeBuffer[i],
                temperatureBuffer[i],
                pressureBuffer[i],
                humidityBuffer[i],
                fixBuffer[i],
                latitudeBuffer[i],
                longitudeBuffer[i],
                speedBuffer[i],
                angleBuffer[i],
                altitudeGPSBuffer[i]);
        Serial.println(printfBuffer);
        index++;

        delay(DELAYTIME);
    }
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    writeToDataBuffer();
    writeToSD();
    if (digitalRead(END_BUTTON) == LOW) {
        exit(0);
    }
    //callToGPS();
    digitalWrite(LED_BUILTIN, LOW);
}