#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <cppQueue.h>
#include <SimpleKalmanFilter.h>


#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1019)

Adafruit_BMP3XX bmp;

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
SimpleKalmanFilter pressureKalmanFilter(0.5, 0.5, 1);

float in = 0;
float sum = 0;
int size_queue = 20;
cppQueue  q(sizeof(in), size_queue, FIFO);  // Instantiate queue


void setup() {
  Serial.begin(9600);
  q.push(&in);    
  
  while (!Serial);

  
  if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  while (1) loop_();
}

void loop_() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  float calc_height_variable = calc_height(bmp.temperature, bmp.pressure / 100.0);
  float height_average_variable = height_average(calc_height_variable);

  if (height_average_variable != 0) {
    float estimated_altitude_average = pressureKalmanFilter.updateEstimate(height_average_variable);
  }
  delay(200);
}

void loop() {}

float height_average(float calc__height){
  
  in = calc__height;
  float out;
  q.push(&(in));
  
  if (q.getCount() < size_queue) {
    sum += in;
  }

  if (q.getCount() == size_queue) {
    sum += in;
    q.pop(&out);
    sum -= out;
    float average = sum /size_queue;
    return average;
    }
    else{
      return 0;  
  }
}

float calc_height(float temp, float pressure) {
  const float P0 = 1023; 
  return ((pow((P0 / pressure), (1/5.257)) - 1) * (1) * (temp + 273.15)) / 0.0065;
}
