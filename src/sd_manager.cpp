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
            "timestamp_utc_str,"           // conservato per compatibilità
            "lat,lon,alt_m,"
            "sat,hdop,"
            "speed_obd_kmh,"
            "acc_lon_G,acc_lat_G,"
            "roll_deg,pitch_deg,slope_deg,slope_confidence,"
            "rpm,load_pct,throttle_pct,"
            "t_mono_us,utc_epoch_us,utc_valid,sync_quality,"
            "imu_t_us,gnss_t_us,obd_speed_t_us,"
            "imu_age_ms,gnss_age_ms,obd_speed_age_ms"
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
    float slopeConfidence,
    int rpm,
    int load,
    float throttle,

    uint64_t monoUs,
    int64_t utcUs,
    bool utcValid,
    uint8_t syncQuality,
    uint64_t imuTimestampUs,
    uint64_t gnssTimestampUs,
    uint64_t obdSpeedTimestampUs,
    int imuAgeMs,
    int gnssAgeMs,
    int obdSpeedAgeMs)
{
    if (!dataFile) return;

    dataFile.print(timestamp);          dataFile.print(",");
    dataFile.print(lat, 6);             dataFile.print(",");
    dataFile.print(lon, 6);             dataFile.print(",");
    dataFile.print(alt, 1);             dataFile.print(",");
    dataFile.print(satellites);         dataFile.print(",");
    dataFile.print(hdop, 2);            dataFile.print(",");
    dataFile.print(speedObd, 1);        dataFile.print(",");
    dataFile.print(lonAcc, 3);          dataFile.print(",");
    dataFile.print(latAcc, 3);          dataFile.print(",");
    dataFile.print(roll, 1);            dataFile.print(",");
    dataFile.print(pitch, 1);           dataFile.print(",");
    dataFile.print(slope, 1);           dataFile.print(",");
    dataFile.print(slopeConfidence, 2); dataFile.print(",");
    dataFile.print(rpm);                dataFile.print(",");
    dataFile.print(load);               dataFile.print(",");
    dataFile.print(throttle, 1);        dataFile.print(",");

    // Nuovi campi temporali
    dataFile.print(monoUs);                 dataFile.print(",");
    dataFile.print(utcUs);                  dataFile.print(",");
    dataFile.print(utcValid ? 1 : 0);       dataFile.print(",");
    dataFile.print(syncQuality);            dataFile.print(",");
    dataFile.print(imuTimestampUs);         dataFile.print(",");
    dataFile.print(gnssTimestampUs);        dataFile.print(",");
    dataFile.print(obdSpeedTimestampUs);    dataFile.print(",");
    dataFile.print(imuAgeMs);               dataFile.print(",");
    dataFile.print(gnssAgeMs);              dataFile.print(",");
    dataFile.println(obdSpeedAgeMs);

    rowCount++;
    if (rowCount % FLUSH_EVERY == 0) dataFile.flush();
}

void sdFlush() {
    if (dataFile) dataFile.flush();
}

void sdClose() {
    if (dataFile) dataFile.close();
}