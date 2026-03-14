// ---------------------------------------------------------------------------
// sd_manager.cpp
// ---------------------------------------------------------------------------

#include "sd_manager.h"
#include <SPI.h>

static const int PIN_SD_CS   = 10;
static const int PIN_SD_SCK  = 12;
static const int PIN_SD_MISO = 13;
static const int PIN_SD_MOSI = 11;

static SPIClass spiSD(FSPI);
static File dataFile;
static int rowCount = 0;
static const int FLUSH_EVERY = 10;

bool sdInit(){
    spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    if (!SD.begin(PIN_SD_CS, spiSD, 1000000)) {
        return false;
    }
    return true;
}

bool sdOpenFile(const char* path) {
    dataFile = SD.open(path, FILE_WRITE);
    if (!dataFile) return false;

    if (dataFile.size() == 0){
        dataFile.println(
            "timestamp,"
            "latitudine,longitudine,altitudine_m,"
            "satelliti,hdop,"
            "velocita_obd_kmh,"
            "accel_lon_G,accel_lat_G,"
            "roll_deg,pitch_deg,slope_deg,slope_reliable,"
            "rpm,load_pct,throttle_pct"
        );
    }
    return true;
}

void sdWriteRow(
    const char* timestamp,
    float lat,  float lon,
    float alt,
    uint8_t satellites,
    float hdop,
    float speedObd,
    float lonAcc, float latAcc,
    float roll,   float pitch,
    float slope,
    uint8_t slopeReliable,
    int rpm,
    int load,
    float throttle)
{
    if (!dataFile) return;

    dataFile.print(timestamp);        dataFile.print(",");
    dataFile.print(lat, 6);           dataFile.print(",");
    dataFile.print(lon, 6);           dataFile.print(",");
    dataFile.print(alt, 1);           dataFile.print(",");
    dataFile.print(satellites);       dataFile.print(",");
    dataFile.print(hdop, 2);          dataFile.print(",");
    dataFile.print(speedObd, 1);      dataFile.print(",");
    dataFile.print(lonAcc, 3);        dataFile.print(",");
    dataFile.print(latAcc, 3);        dataFile.print(",");
    dataFile.print(roll, 1);          dataFile.print(",");
    dataFile.print(pitch, 1);         dataFile.print(",");
    dataFile.print(slope, 1);         dataFile.print(",");
    dataFile.print(slopeReliable);    dataFile.print(",");
    dataFile.print(rpm);              dataFile.print(",");
    dataFile.print(load);             dataFile.print(",");
    dataFile.println(throttle, 1);

    rowCount++;
    if (rowCount % FLUSH_EVERY == 0) dataFile.flush();
}

void sdFlush() {
    if (dataFile) dataFile.flush();
}

void sdClose() {
    if (dataFile) dataFile.close();
}