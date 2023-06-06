#include <Arduino.h>
#include <SD.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GPS.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define DELAYTIME 100
#define CHIPSELECT 4
#define DATABUFFERSIZE 50
#define GPSSerial Serial1
#define GPSECHO false

/*
double altitudeBuffer[DATABUFFERSIZE] = {};
double temperatureBuffer[DATABUFFERSIZE] = {};
double pressureBuffer[DATABUFFERSIZE] = {};
double humidityBuffer[DATABUFFERSIZE] = {};
double fixBuffer[DATABUFFERSIZE] = {};
double locationNSBuffer[DATABUFFERSIZE] = {};
double locationEWBuffer[DATABUFFERSIZE] = {};
double speedBuffer[DATABUFFERSIZE] = {};
double angleBuffer[DATABUFFERSIZE] = {};
double altitudeGPSBuffer[DATABUFFERSIZE] = {};
*/

char printfBuffer[50];
const int START_BUTTON = 7;
const int END_BUTTON = 3;
uint32_t timer = millis();

Adafruit_BME280 bme; // I2C
Adafruit_GPS GPS(&GPSSerial);

struct SALTData {
    double SALTfix;
    double SALTlocationNS;
    double SALTlocationEW;
    double SALTspeed;
    double SALTangle;
    double SALTaltitudeGPS;
    double SALTaltitude;
    double SALTpressure;
    double SALTtemp;
    double SALThumidity;
};

SALTData *dataBuffer[DATABUFFERSIZE];

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
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.println("Altitude (m), Temperature (C), Pressure (kpa), Humidity (%)");
        dataFile.close();
        Serial.println("Header Created");
    }
    else {
        Serial.println("error opening datalog.csv");
        exit(0);
    }

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
        for (SALTData *data : dataBuffer) {
            sprintf(printfBuffer, "%.2f, %.2f, %.2f, %.2f, %.2f",
                    data->SALTaltitude,
                    data->SALTtemp,
                    data->SALTpressure,
                    data->SALThumidity,
                    data->SALTfix);
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
    int index = 0;
    for (SALTData *data : dataBuffer) {
        data->SALTaltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
        data->SALTtemp = bme.readTemperature();
        data->SALTpressure = bme.readPressure() / 1000.0F;
        data->SALThumidity = bme.readHumidity();
        data->SALTfix = (int)GPS.fix;
        sprintf(printfBuffer, "%d: %.2f m, %.2f C, %.2f kpa, %.2f %%",
                index,
                data->SALTaltitude,
                data->SALTtemp,
                data->SALTpressure,
                data->SALThumidity);
        Serial.println(printfBuffer);
        index++;

        delay(DELAYTIME);
    }
}

void callToGPS() // run over and over again
{
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
        if (c) Serial.print(c);
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return; // we can fail to parse a sentence in which case we should just wait for another
    }

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000) {
        timer = millis(); // reset the timer
        Serial.print("\nTime: ");
        if (GPS.hour < 10) { Serial.print('0'); }
        Serial.print(GPS.hour, DEC); Serial.print(':');
        if (GPS.minute < 10) { Serial.print('0'); }
        Serial.print(GPS.minute, DEC); Serial.print(':');
        if (GPS.seconds < 10) { Serial.print('0'); }
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        if (GPS.milliseconds < 10) {
            Serial.print("00");
        } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
            Serial.print("0");
        }
        Serial.println(GPS.milliseconds);
        /*Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);*/
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        if (GPS.fix) {
            Serial.print("Location: ");
            Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
            Serial.print(", ");
            Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
            Serial.print("Speed (knots): "); Serial.println(GPS.speed);
            Serial.print("Angle: "); Serial.println(GPS.angle);
            Serial.print("Altitude: "); Serial.println(GPS.altitude);
            Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
            Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
        }
    }
}

void loop() {
    writeToDataBuffer();
    writeToSD();
    if (digitalRead(END_BUTTON) == LOW) {
        exit(0);
    }
    //callToGPS();
}