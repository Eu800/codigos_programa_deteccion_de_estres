#include <Wire.h>
#include "MAX30105.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Configuración de pines para ECG
const int loPlusPin = 10;
const int loMinusPin = 11;
const int ecgPin = A0;  // Pin de lectura de la señal ECG

// Configuración del sensor MAX30105 para BVP
MAX30105 particleSensor;

// Variables para sincronización
volatile bool sampleReady = false; // Bandera para indicar que se debe tomar una muestra
long irValue = 0; // Valor IR de MAX30105 (BVP)
int ecgValue = 0; // Valor ECG

// Configuración del filtro para BVP
const int filterOrder = 32; // Orden del filtro
float filterCoefficients[filterOrder] = {
    -0.0035, -0.0037, -0.0040, -0.0043, -0.0047, -0.0051, -0.0056, -0.0062,
    -0.0068, -0.0075, -0.0083, -0.0091, -0.0099, -0.0108, -0.0116, -0.0125,
    0.9875, -0.0125, -0.0116, -0.0108, -0.0099, -0.0091, -0.0083, -0.0075,
    -0.0068, -0.0062, -0.0056, -0.0051, -0.0047, -0.0043, -0.0040, -0.0037
};
float signalBuffer[filterOrder] = {0}; // Buffer circular para el filtro
int bufferIndex = 0;

void setup() {
  Serial.begin(115200); // Comunicación serial

  // Configurar pines LO+ y LO-
  pinMode(loPlusPin, INPUT);
  pinMode(loMinusPin, INPUT);

  // Configurar el temporizador para 1000 Hz
  cli(); // Deshabilitar interrupciones globales
  TCCR1A = 0; // Limpiar registros de control del temporizador 1
  TCCR1B = 0;
  TCNT1 = 0; // Reiniciar el contador
  OCR1A = 15999; // Comparación para 1 ms (16 MHz / 1000 Hz - 1)
  TCCR1B |= (1 << WGM12); // Modo CTC
  TCCR1B |= (1 << CS10);  // Prescaler de 1
  TIMSK1 |= (1 << OCIE1A); // Habilitar interrupción por comparación A
  sei(); // Habilitar interrupciones globales

  // Configuración del sensor MAX30105 (para BVP)
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }

  byte ledBrightness = 0x1F; // Brillo del LED
  byte sampleAverage = 1;   // Promedio de muestras
  byte ledMode = 2;         // LEDs activados (IR)
  int sampleRate = 1000;    // Frecuencia de muestreo
  int pulseWidth = 411;     // Duración del pulso
  int adcRange = 4096;      // Rango ADC

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void loop() {
  // Si hay una muestra lista
  if (sampleReady) {
    sampleReady = false; // Reiniciar la bandera

    // Leer la señal ECG
    ecgValue = analogRead(ecgPin);

    // Leer el valor IR del sensor MAX30105 (BVP)
    irValue = particleSensor.getIR();

    // Actualiza el buffer circular para el filtro pasa-banda de BVP
    signalBuffer[bufferIndex] = irValue;
    bufferIndex = (bufferIndex + 1) % filterOrder;

    // Aplicar el filtro pasa-banda a la señal IR para obtener el valor BVP
    float bvpValue = 0;
    for (int i = 0; i < filterOrder; i++) {
      int index = (bufferIndex + i) % filterOrder;
      bvpValue += signalBuffer[index] * filterCoefficients[i];
    }

    // Normalizar las señales ECG y BVP
    float normalizedEcg = (float)(ecgValue - 512) / 512.0;  // Normalizando a un rango de -1 a 1
    float normalizedBvp = (float)(bvpValue - 1000) / 1000.0; // Normalizando a un rango de -1 a 1

    // Enviar los valores por el monitor serial
    Serial.print(normalizedEcg);   // Valor ECG normalizado
    Serial.print(",");             // Separador
    Serial.println(normalizedBvp); // Valor BVP normalizado
  }
}

// Interrupción del temporizador (sincroniza la lectura)
ISR(TIMER1_COMPA_vect) {
  sampleReady = true; // Indicar que se debe tomar una muestra
}
