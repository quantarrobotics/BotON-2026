#include "MatrixMiniR4.h"

// =====================================================
// CALIBRACION DE AZUL - MXColor V3
// =====================================================
//
// Este programa sirve para encontrar un umbral confiable
// para detectar el color AZUL antes de entrar al dojo.
//
// SENSOR DE COLOR:
// I2C4 - MXColorV3
//
// FUNCIONAMIENTO:
// 1. Enciende el robot.
// 2. Abre el Monitor Serial a 9600 baudios.
// 3. Coloca el sensor sobre la zona AZUL que utilizara
//    el robot para entrar al dojo.
// 4. Presiona el boton UP.
// 5. El robot toma varias muestras durante 5 segundos.
// 6. Al terminar muestra los valores H, S y V obtenidos.
// 7. Tambien calcula un umbral recomendado para el azul.
//
// NO mueve los motores.
// =====================================================

bool sensorColorOK = false;

// Cantidad de muestras durante la calibracion.
const int NUM_MUESTRAS = 100;

// Tiempo total de calibracion.
const unsigned long TIEMPO_CALIBRACION = 5000;

// Margenes de seguridad para evitar que el umbral quede
// demasiado ajustado a una sola lectura.
const float MARGEN_H = 5.0;
const float MARGEN_S = 10.0;
const float MARGEN_V = 10.0;

// =====================================================
// VARIABLES DE CALIBRACION
// =====================================================

float hMin = 360.0;
float hMax = 0.0;
float sMin = 100.0;
float sMax = 0.0;
float vMin = 100.0;
float vMax = 0.0;

float hSuma = 0.0;
float sSuma = 0.0;
float vSuma = 0.0;

int muestrasValidas = 0;


// =====================================================
// MOSTRAR UNA LECTURA
// =====================================================

void mostrarLectura(float H, float S, float V)
{
  Serial.print("H: ");
  Serial.print(H, 1);
  Serial.print(" | S: ");
  Serial.print(S, 1);
  Serial.print(" | V: ");
  Serial.println(V, 1);
}


// =====================================================
// CALIBRAR AZUL
// =====================================================

void calibrarAzul()
{
  hMin = 360.0;
  hMax = 0.0;
  sMin = 100.0;
  sMax = 0.0;
  vMin = 100.0;
  vMax = 0.0;

  hSuma = 0.0;
  sSuma = 0.0;
  vSuma = 0.0;

  muestrasValidas = 0;

  Serial.println();
  Serial.println("============================================");
  Serial.println("          CALIBRANDO COLOR AZUL");
  Serial.println("============================================");
  Serial.println("NO muevas el robot.");
  Serial.println("Mantén el sensor sobre el AZUL.");
  Serial.println("Tomando muestras durante 5 segundos...");
  Serial.println();

  unsigned long inicio = millis();
  unsigned long ultimaMuestra = 0;
  int numeroMuestra = 0;

  while ((unsigned long)(millis() - inicio) < TIEMPO_CALIBRACION)
  {
    if ((unsigned long)(millis() - ultimaMuestra) >= 45 &&
        numeroMuestra < NUM_MUESTRAS)
    {
      ultimaMuestra = millis();

      float H = MiniR4.I2C4.MXColorV3.getH();
      float S = MiniR4.I2C4.MXColorV3.getS();
      float V = MiniR4.I2C4.MXColorV3.getV();

      // Guardar extremos.
      if (H < hMin) hMin = H;
      if (H > hMax) hMax = H;

      if (S < sMin) sMin = S;
      if (S > sMax) sMax = S;

      if (V < vMin) vMin = V;
      if (V > vMax) vMax = V;

      hSuma += H;
      sSuma += S;
      vSuma += V;

      muestrasValidas++;
      numeroMuestra++;

      // Mostrar algunas lecturas para comprobar que el sensor
      // esta leyendo correctamente.
      if (numeroMuestra % 10 == 0)
      {
        Serial.print("Muestra ");
        Serial.print(numeroMuestra);
        Serial.print(" -> ");
        mostrarLectura(H, S, V);
      }
    }

    delay(2);
  }

  if (muestrasValidas == 0)
  {
    Serial.println();
    Serial.println("ERROR: No se obtuvieron muestras.");
    return;
  }

  // Promedios.
  float hPromedio = hSuma / muestrasValidas;
  float sPromedio = sSuma / muestrasValidas;
  float vPromedio = vSuma / muestrasValidas;

  // ===================================================
  // CALCULAR UMBRAL RECOMENDADO
  // ===================================================
  //
  // Se utiliza el rango real medido y se agrega un pequeño
  // margen para compensar cambios de luz.
  // ===================================================

  float azulHMin = hMin - MARGEN_H;
  float azulHMax = hMax + MARGEN_H;

  float azulSMin = sMin - MARGEN_S;
  float azulVMin = vMin - MARGEN_V;

  // Limitar valores a rangos validos.
  if (azulHMin < 0) azulHMin = 0;
  if (azulHMax > 360) azulHMax = 360;

  if (azulSMin < 0) azulSMin = 0;
  if (azulSMin > 100) azulSMin = 100;

  if (azulVMin < 0) azulVMin = 0;
  if (azulVMin > 100) azulVMin = 100;

  // ===================================================
  // RESULTADOS
  // ===================================================

  Serial.println();
  Serial.println("============================================");
  Serial.println("             RESULTADO AZUL");
  Serial.println("============================================");

  Serial.print("Muestras validas: ");
  Serial.println(muestrasValidas);

  Serial.println();
  Serial.println("RANGO MEDIDO:");

  Serial.print("H: ");
  Serial.print(hMin, 1);
  Serial.print(" - ");
  Serial.println(hMax, 1);

  Serial.print("S: ");
  Serial.print(sMin, 1);
  Serial.print(" - ");
  Serial.println(sMax, 1);

  Serial.print("V: ");
  Serial.print(vMin, 1);
  Serial.print(" - ");
  Serial.println(vMax, 1);

  Serial.println();
  Serial.println("PROMEDIO:");

  Serial.print("H promedio: ");
  Serial.println(hPromedio, 1);

  Serial.print("S promedio: ");
  Serial.println(sPromedio, 1);

  Serial.print("V promedio: ");
  Serial.println(vPromedio, 1);

  Serial.println();
  Serial.println("============================================");
  Serial.println("       UMBRAL RECOMENDADO PARA TU CODIGO");
  Serial.println("============================================");

  Serial.print("float AZUL_H_MIN = ");
  Serial.print(azulHMin, 1);
  Serial.println(";");

  Serial.print("float AZUL_H_MAX = ");
  Serial.print(azulHMax, 1);
  Serial.println(";");

  Serial.print("float AZUL_S_MIN = ");
  Serial.print(azulSMin, 1);
  Serial.println(";");

  Serial.print("float AZUL_V_MIN = ");
  Serial.print(azulVMin, 1);
  Serial.println(";");

  Serial.println();
  Serial.println("COPIA ESTOS 4 VALORES EN TU PROGRAMA PRINCIPAL.");
  Serial.println("============================================");

  Serial.println();
  Serial.println("Ejemplo de la condicion de deteccion:");
  Serial.println("H >= AZUL_H_MIN && H <= AZUL_H_MAX");
  Serial.println("S >= AZUL_S_MIN && V >= AZUL_V_MIN");

  Serial.println();
  Serial.println("Calibracion terminada.");
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  MiniR4.begin();
  MiniR4.PWR.setBattCell(2);

  Serial.begin(9600);

  sensorColorOK =
    MiniR4.I2C4.MXColorV3.begin();

  Serial.println();
  Serial.println("============================================");
  Serial.println("       CALIBRADOR DE AZUL - DOJO");
  Serial.println("============================================");

  if (sensorColorOK)
  {
    Serial.println("Sensor de color MXColor V3: OK");
  }
  else
  {
    Serial.println("ERROR: No se pudo iniciar MXColor V3.");
    while (true)
    {
      delay(100);
    }
  }

  Serial.println();
  Serial.println("Coloca el sensor sobre el color AZUL.");
  Serial.println("Cuando estes listo, presiona el boton UP.");

  // Esperar boton UP.
  while (!MiniR4.BTN_UP.getState())
  {
    delay(10);
  }

  delay(300);

  calibrarAzul();
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // Mostrar lecturas actuales despues de calibrar.
  // Esto permite comparar el azul calibrado con otros colores.

  float H = MiniR4.I2C4.MXColorV3.getH();
  float S = MiniR4.I2C4.MXColorV3.getS();
  float V = MiniR4.I2C4.MXColorV3.getV();

  Serial.print("ACTUAL -> ");
  mostrarLectura(H, S, V);

  delay(500);
}
