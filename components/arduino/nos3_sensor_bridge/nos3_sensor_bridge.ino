/*
** NOS3 Sensor Bridge - Arduino Sketch
** Reads MPU6050 (IMU) and LDR (light) sensors, sends data over Serial
** Protocol:
**   T,<temp_c>\n                         (Temperature: Celsius)
**   I,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n   (IMU: m/s², deg/s)
**   L,<ldr0>\n     (LDR: 0.0-1.0 normalized)
*/

#include <Wire.h>

const uint8_t MPU6050_ADDR = 0x68;
const int LDR_PINS[] = {A0, A1, A2, A3};
const int NUM_LDRS = 1;

const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE  = 131.0;
const float G_TO_MS2    = 9.80665;

/* Low-pass filter katsayisi: 0.0-1.0 arasi, kucuk = daha yumusak */
const float ALPHA = 0.15;

/* Kalibrasyon offsetleri */
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float ax_offset = 0, ay_offset = 0;

/* Filtrelenmis degerler */
float fax = 0, fay = 0, faz = 9.81;
float fgx = 0, fgy = 0, fgz = 0;
float ftemp = 25.0;
bool first_sample = true;

void readRaw(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz, int16_t &temp)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
    ax   = (Wire.read() << 8) | Wire.read();
    ay   = (Wire.read() << 8) | Wire.read();
    az   = (Wire.read() << 8) | Wire.read();
    temp = (Wire.read() << 8) | Wire.read();
    gx   = (Wire.read() << 8) | Wire.read();
    gy   = (Wire.read() << 8) | Wire.read();
    gz   = (Wire.read() << 8) | Wire.read();
}

void calibrate()
{
    const int SAMPLES = 300;
    long ax_sum=0, ay_sum=0, gx_sum=0, gy_sum=0, gz_sum=0;
    int16_t ax,ay,az,gx,gy,gz,temp;

    for (int i = 0; i < SAMPLES; i++)
    {
        readRaw(ax, ay, az, gx, gy, gz, temp);
        ax_sum += ax; ay_sum += ay;
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        delay(5);
    }

    gx_offset = (gx_sum / SAMPLES) / GYRO_SCALE;
    gy_offset = (gy_sum / SAMPLES) / GYRO_SCALE;
    gz_offset = (gz_sum / SAMPLES) / GYRO_SCALE;
    ax_offset = (ax_sum / SAMPLES) / ACCEL_SCALE * G_TO_MS2;
    ay_offset = (ay_sum / SAMPLES) / ACCEL_SCALE * G_TO_MS2;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000); /* 400kHz I2C - daha stabil okuma */

    /* Wake MPU6050 */
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); Wire.write(0x01); /* PLL with X gyro ref */
    Wire.endTransmission(true);

    /* DLPF: 94Hz bandwidth - donanim seviyesinde noise azaltma */
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A); Wire.write(0x02); /* CONFIG: DLPF_CFG=2 */
    Wire.endTransmission(true);

    /* Accelerometer: +-2g */
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C); Wire.write(0x00);
    Wire.endTransmission(true);

    /* Gyroscope: +-250 deg/s */
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); Wire.write(0x00);
    Wire.endTransmission(true);

    delay(200);

    /* Kalibrasyon - Arduino'yu duz ve sabit tut */
    calibrate();
}

void loop()
{
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, temp_raw;
    readRaw(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, temp_raw);

    /* Fiziksel birimlere cevir */
    float ax = (ax_raw / ACCEL_SCALE) * G_TO_MS2 - ax_offset;
    float ay = (ay_raw / ACCEL_SCALE) * G_TO_MS2 - ay_offset;
    float az = (az_raw / ACCEL_SCALE) * G_TO_MS2;
    float gx = gx_raw / GYRO_SCALE - gx_offset;
    float gy = gy_raw / GYRO_SCALE - gy_offset;
    float gz = gz_raw / GYRO_SCALE - gz_offset;
    float temp_c = temp_raw / 340.0 + 36.53;

    /* Low-pass filter uygula */
    if (first_sample)
    {
        fax = ax; fay = ay; faz = az;
        fgx = gx; fgy = gy; fgz = gz;
        ftemp = temp_c;
        first_sample = false;
    }
    else
    {
        fax = ALPHA * ax + (1.0 - ALPHA) * fax;
        fay = ALPHA * ay + (1.0 - ALPHA) * fay;
        faz = ALPHA * az + (1.0 - ALPHA) * faz;
        fgx = ALPHA * gx + (1.0 - ALPHA) * fgx;
        fgy = ALPHA * gy + (1.0 - ALPHA) * fgy;
        fgz = ALPHA * gz + (1.0 - ALPHA) * fgz;
        ftemp = 0.05 * temp_c + 0.95 * ftemp;
    }

    /* Gonder */
    Serial.print("T,"); Serial.println(ftemp, 2);
    Serial.flush();
    delay(20);

    Serial.print("I,");
    Serial.print(fax, 3); Serial.print(",");
    Serial.print(fay, 3); Serial.print(",");
    Serial.print(faz, 3); Serial.print(",");
    Serial.print(fgx, 3); Serial.print(",");
    Serial.print(fgy, 3); Serial.print(",");
    Serial.println(fgz, 3);
    Serial.flush();

    delay(20);

    Serial.print("L,");
    for (int i = 0; i < NUM_LDRS; i++)
    {
        Serial.print(1.0 - (analogRead(LDR_PINS[i]) / 1023.0), 3);
        if (i < NUM_LDRS - 1) Serial.print(",");
    }
    Serial.println();
    Serial.flush();

    delay(20);
}
