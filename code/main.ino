#include "MatrixMiniR4.h"

// =====================================================
// ROBOT / VERSION: DOJO 6 V3 - PID RAPIDO
// =====================================================

// =====================================================
// CONFIGURACIÓN DE MOTORES
// =====================================================

int motorIzquierdo = 1;
int motorDerecho   = 2;

bool invertirIzquierdo = false;
bool invertirDerecho   = true;


// =====================================================
// FAST LINE
// =====================================================

float velocidad = 100;

// PID optimizado para mayor velocidad y respuesta rápida.
// Kp más alto = corrige antes en curvas.
// Kd más bajo = menos frenadas bruscas por ruido del sensor.
float Kp = 21.0;
float Kd = 26.0;

// Umbral
float NEGRO = 55;

float errorAnterior = 0;


// =====================================================
// SENSOR DE COLOR V3
// =====================================================

bool sensorColorOK = false;

float AZUL_H_MIN = 200.0;
float AZUL_H_MAX = 220.0;

float AZUL_S_MIN = 45.0;
float AZUL_V_MIN = 20.0;


// =====================================================
// LASER V2
// =====================================================
//
// LASER 1 = I2C1
// LASER 2 = I2C2
//
// Distancia máxima de detección:
// 350 mm = 35 cm
// =====================================================

const int DISTANCIA_OBJETO = 450;

// Distancia ampliada durante una búsqueda prolongada.
// 550 mm = 55 cm.
const int DISTANCIA_BUSQUEDA_PROLONGADA = 1000;

// Si supera 30 mm consideramos que el objeto
// ya no está delante.
const int OBJETO_ALEJADO = 450;

// =====================================================
// BÚSQUEDA PROLONGADA
// =====================================================
//
// Si el robot lleva más de 1.5 segundos buscando sin
// encontrar un objeto, los láseres amplían su rango
// de detección de 45 cm a 100 cm.
// Al detectar algo, vuelve inmediatamente a 35 cm.
// =====================================================

const unsigned long TIEMPO_BUSQUEDA_PROLONGADA = 1500;

unsigned long inicioBusqueda = 0;

// =====================================================
// AVANCE DESPUES DE LOS GIROS DE 180 GRADOS
// =====================================================
// Secuencia de búsqueda:
// 180° -> 20 cm -> 360° -> 180° -> 20 cm
// -> 180° -> 20 cm -> 360° -> repetir.
//
// Los giros de 360° no llevan avance.
// =====================================================
const unsigned long TIEMPO_GIRO_180 = 450;
const unsigned long TIEMPO_GIRO_360 = TIEMPO_GIRO_180 * 2;
const unsigned long TIEMPO_AVANCE_20CM = 360;

// Secuencia de búsqueda de 5 etapas:
// 0 = 180° + 20 cm
// 1 = 360° sin avance
// 2 = 180° + 20 cm
// 3 = 180° + 20 cm
// 4 = 360° sin avance
int etapaGiroBusqueda = 0;
bool avanceDespuesDeGiro = false;

// Indica si la búsqueda ya superó los 3 segundos.
bool busquedaProlongada = false;

int distanciaDeteccionActual = DISTANCIA_OBJETO;


// ============================ =========================
// VELOCIDADES DEL DOJO
// =====================================================

const int VELOCIDAD_CENTRO = 45;

const int VELOCIDAD_BUSQUEDA = 35;

const int VELOCIDAD_ATAQUE = 65;

const int VELOCIDAD_ESCAPE = 79;


// =====================================================
// TIEMPO PARA LLEGAR AL CENTRO
// =====================================================

const unsigned long TIEMPO_CENTRO = 1800;


// =====================================================
// MODO DOJO
// =====================================================

bool modoDojo = false;


// =====================================================
// ESTADOS
// =====================================================

enum EstadoRobot
{
  SEGUIR_LINEA,
  IR_CENTRO,
  BUSCAR_OBJETO,
  ATACAR_OBJETO,
  ESCAPAR_BORDE
};

EstadoRobot estado = SEGUIR_LINEA;


// =====================================================
// CONFIGURAR MOTORES
// =====================================================

void configurarMotores()
{
  MiniR4.M1.setPPR_RPM(545, 200);
  MiniR4.M1.setReverse(invertirIzquierdo);

  MiniR4.M2.setPPR_RPM(545, 200);
  MiniR4.M2.setReverse(invertirDerecho);

  MiniR4.DriveDC.begin(
    motorIzquierdo,
    motorDerecho,
    invertirIzquierdo,
    invertirDerecho
  );

  MiniR4.DriveDC.Move(0, 0);
}


// =====================================================
// DETECTAR AZUL
// =====================================================

bool detectaAzul()
{
  if (!sensorColorOK)
    return false;

  float H = MiniR4.I2C4.MXColorV3.getH();
  float S = MiniR4.I2C4.MXColorV3.getS();
  float V = MiniR4.I2C4.MXColorV3.getV();

  if (
    H >= AZUL_H_MIN &&
    H <= AZUL_H_MAX &&
    S >= AZUL_S_MIN &&
    V >= AZUL_V_MIN
  )
  {
    return true;
  }

  return false;
}


// =====================================================
// DETECTAR FIN DE PISTA
// =====================================================
//
// ESTA FUNCIÓN SOLO SE USA EN LA PISTA.
// No se utiliza dentro del dojo.
// =====================================================

bool todosNegros()
{
  int sensoresNegros = 0;

  for (int i = 0; i < 10; i++)
  {
    float valor =
      MiniR4.I2C0.MXLineTracer.getSensor(i);

    if (valor >= NEGRO)
    {
      sensoresNegros++;
    }
  }

  return sensoresNegros == 10;
}


// =====================================================
// DETECTAR BORDE DEL DOJO
// =====================================================
//
// CONFIGURACIÓN FÍSICA:
//
//       INTERIOR = NEGRO
//       BORDE    = BLANCO
//
// Según el comportamiento del sensor:
//
//       valor < 55  = NEGRO = INTERIOR
//       valor >= 55 = BLANCO = BORDE
//
// Si 3 o más sensores detectan el borde,
// se activa el escape.
// =====================================================

bool detectaBorde()
{
  int sensoresBorde = 0;

  for (int i = 0; i < 10; i++)
  {
    float valor =
      MiniR4.I2C0.MXLineTracer.getSensor(i);

    // BLANCO / BORDE
    if (valor >= NEGRO)
    {
      sensoresBorde++;
    }
  }

  if (sensoresBorde >= 10)
  {
    return true;
  }

  return false;
}


// =====================================================
// DETECTAR BORDE DURANTE ATAQUE
// =====================================================
//
// Mientras el robot esta empujando, se prioriza la
// seguridad: con 3 sensores o mas detectando blanco
// se ordena escapar inmediatamente.
// =====================================================

bool detectaBordeDuranteAtaque()
{
  int sensoresBorde = 0;

  for (int i = 0; i < 10; i++)
  {
    float valor =
      MiniR4.I2C0.MXLineTracer.getSensor(i);

    if (valor >= NEGRO)
    {
      sensoresBorde++;
    }
  }

  return sensoresBorde >= 1;
}


// =====================================================
// FAST LINE
// =====================================================

void fastLine()
{
  float error =
    MiniR4.I2C0.MXLineTracer.getError();

  // Derivada limitada para evitar que una lectura aislada
  // provoque una frenada excesiva a alta velocidad.
  float derivada = error - errorAnterior;
  derivada = constrain(derivada, -25, 25);

  float correccion =
    (error * Kp) +
    (derivada * Kd);

  // Mantener máxima velocidad en recta y permitir una
  // reducción controlada cuando la curva exige mucha corrección.
  float velocidadActual = velocidad;
  if (abs(error) > 25)
    velocidadActual = 90;
  if (abs(error) > 45)
    velocidadActual = 80;

  float motorIzq =
    velocidadActual + correccion;

  float motorDer =
    velocidadActual - correccion;

  motorIzq =
    constrain(motorIzq, -100, 100);

  motorDer =
    constrain(motorDer, -100, 100);

  MiniR4.DriveDC.Move(
    motorIzq,
    motorDer
  );

  errorAnterior = error;
}


// =====================================================
// LEER LASER 1
// =====================================================
//
// Laser frontal izquierdo
// Puerto I2C1
// =====================================================

int distanciaLaser1()
{
  int distancia =
    MiniR4.I2C1.MXLaserV2.getDistance();

  if (distancia >= 2000)
    return 2000;

  return distancia;
}


// =====================================================
// LEER LASER 2
// =====================================================
//
// Laser frontal derecho
// Puerto I2C2
// =====================================================

int distanciaLaser2()
{
  int distancia =
    MiniR4.I2C2.MXLaserV2.getDistance();

  if (distancia >= 2000)
    return 2000;

  return distancia;
}


// =====================================================
// DETECTAR OBJETO CON LOS DOS LASER
// =====================================================

bool objetoDetectado(int distancia1, int distancia2)
{
  bool laser1Detecta =
    (distancia1 >= 45 &&
     distancia1 <= distanciaDeteccionActual);

  bool laser2Detecta =
    (distancia2 >= 45 &&
     distancia2 <= distanciaDeteccionActual);

  return laser1Detecta || laser2Detecta;
}


// =====================================================
// INICIAR BÚSQUEDA
// =====================================================

void iniciarBusqueda()
{
  estado = BUSCAR_OBJETO;

  // Reiniciar completamente el temporizador de búsqueda.
  inicioBusqueda = millis();
  busquedaProlongada = false;

  // Cada nueva búsqueda comienza con la distancia normal.
  distanciaDeteccionActual = DISTANCIA_OBJETO;

  etapaGiroBusqueda = 0;
  avanceDespuesDeGiro = false;
}


// =====================================================
// DETENER
// =====================================================

void parar()
{
  MiniR4.DriveDC.Move(0, 0);

  delay(30);
}


// =====================================================
// IR AL CENTRO DEL DOJO
// =====================================================

void irCentro()
{
  Serial.println();
  Serial.println("==============================");
  Serial.println("       ENTRANDO AL DOJO");
  Serial.println("==============================");

  Serial.println("Avanzando hacia el centro...");


  // ---------------------------------------------------
  // AVANZAR
  // ---------------------------------------------------

  MiniR4.DriveDC.Move(
    VELOCIDAD_CENTRO,
    VELOCIDAD_CENTRO
  );

  delay(TIEMPO_CENTRO);


  parar();

  delay(100);


  Serial.println(">>> CENTRO DEL DOJO <<<");
  Serial.println(">>> INTERIOR = NEGRO <<<");
  Serial.println(">>> BORDE = BLANCO <<<");


  // ---------------------------------------------------
  // ACTIVAR MODO DOJO
  // ---------------------------------------------------

  modoDojo = true;

  iniciarBusqueda();
}


// =====================================================
// BUSCAR OBJETO
// =====================================================
//
// El robot gira sobre su propio eje.
// Los dos Laser buscan objetos incluso mientras el robot esta girando.
// =====================================================

void buscarObjeto()
{
  int distancia1 = distanciaLaser1();
  int distancia2 = distanciaLaser2();


  // ---------------------------------------------------
  // BÚSQUEDA PROLONGADA
  // ---------------------------------------------------
  // Si lleva 1.5 segundos completos buscando sin detectar
  // un objeto, ampliar la detección de 45 cm a 100 cm.
  // Este cambio se hace una sola vez por cada búsqueda.
  if (
    !busquedaProlongada &&
    (unsigned long)(millis() - inicioBusqueda) >= TIEMPO_BUSQUEDA_PROLONGADA
  )
  {
    busquedaProlongada = true;
    distanciaDeteccionActual = DISTANCIA_BUSQUEDA_PROLONGADA;

    Serial.println(">>> BUSQUEDA > 1.5 SEGUNDOS <<<");
    Serial.println(">>> DETECCION AMPLIADA A 1 METRO <<<");
  }


  Serial.print("BUSQUEDA - Laser 1: ");
  Serial.print(distancia1);
  Serial.print(" mm | Laser 2: ");
  Serial.print(distancia2);
  Serial.println(" mm");


  // ===================================================
  // PRIORIDAD 1: BORDE
  // ===================================================

  if (detectaBorde())
  {
    Serial.println(">>> BORDE BLANCO DETECTADO <<<");

    estado = ESCAPAR_BORDE;

    return;
  }


  // ===================================================
  // PRIORIDAD 2: OBJETO
  // ===================================================

  if (objetoDetectado(distancia1, distancia2))
  {
    Serial.println(">>> OBJETO DETECTADO <<<");

    // Volver inmediatamente a la detección normal.
    distanciaDeteccionActual = DISTANCIA_OBJETO;
    busquedaProlongada = false;

    estado = ATACAR_OBJETO;

    return;
  }


  // ===================================================
  // BÚSQUEDA POR SECUENCIA DE GIROS
  // ===================================================
  // Secuencia exacta:
  // 180° + 20 cm
  // 360° sin avance
  // 180° + 20 cm
  // 180° + 20 cm
  // 360° sin avance
  // y repetir.
  if (!avanceDespuesDeGiro)
  {
    unsigned long tiempoGiro;
    int gradosGiro;

    if (etapaGiroBusqueda == 0)
    {
      tiempoGiro = TIEMPO_GIRO_180;
      gradosGiro = 180;
    }
    else if (etapaGiroBusqueda == 1)
    {
      tiempoGiro = TIEMPO_GIRO_360;
      gradosGiro = 360;
    }
    else if (etapaGiroBusqueda == 2)
    {
      tiempoGiro = TIEMPO_GIRO_180;
      gradosGiro = 180;
    }
    else if (etapaGiroBusqueda == 3)
    {
      tiempoGiro = TIEMPO_GIRO_180;
      gradosGiro = 180;
    }
    else
    {
      tiempoGiro = TIEMPO_GIRO_360;
      gradosGiro = 360;
    }

    Serial.print(">>> GIRANDO ");
    Serial.print(gradosGiro);
    Serial.println(" GRADOS <<<");

    MiniR4.DriveDC.Move(
      VELOCIDAD_BUSQUEDA,
      -VELOCIDAD_BUSQUEDA
    );

    // Durante TODO el giro se vigilan continuamente los láseres
    // y los sensores de borde. Los delay() aquí bloqueaban la
    // reacción y podían hacer que el robot pasara de largo.
    unsigned long inicioGiro = millis();
    bool objetivoDuranteGiro = false;
    bool bordeDuranteGiro = false;

    while ((unsigned long)(millis() - inicioGiro) < tiempoGiro)
    {
      int giroLaser1 = distanciaLaser1();
      int giroLaser2 = distanciaLaser2();

      // El borde tiene prioridad absoluta.
      if (detectaBorde())
      {
        bordeDuranteGiro = true;
        break;
      }

      // Si cualquiera de los láseres encuentra el objeto,
      // cortar el giro inmediatamente y pasar a atacarlo.
      if (objetoDetectado(giroLaser1, giroLaser2))
      {
        objetivoDuranteGiro = true;
        break;
      }

      delay(5);
    }

    parar();

    if (bordeDuranteGiro)
    {
      Serial.println(">>> BORDE DETECTADO DURANTE GIRO: ESCAPE INMEDIATO <<<");
      estado = ESCAPAR_BORDE;
      return;
    }

    if (objetivoDuranteGiro)
    {
      Serial.println(">>> LASER DETECTO OBJETO DURANTE GIRO <<<");
      Serial.println(">>> GIRO INTERRUMPIDO: BUSCANDO/ATACANDO OBJETO <<<");
      distanciaDeteccionActual = DISTANCIA_OBJETO;
      busquedaProlongada = false;
      estado = ATACAR_OBJETO;
      return;
    }

    Serial.println(">>> GIRO COMPLETADO <<<");

    // Los giros de 180 grados van seguidos de 20 cm.
    if (gradosGiro == 180)
    {
      Serial.println(">>> AVANZANDO 20 CM <<<");
      avanceDespuesDeGiro = true;
    }
    else
    {
      // Los giros de 360 grados NO llevan avance.
      Serial.println(">>> 360 GRADOS: SIN AVANCE <<<");
      avanceDespuesDeGiro = false;

      etapaGiroBusqueda++;

      if (etapaGiroBusqueda > 4)
        etapaGiroBusqueda = 0;
    }
  }
  else
  {
    MiniR4.DriveDC.Move(
      VELOCIDAD_BUSQUEDA,
      VELOCIDAD_BUSQUEDA
    );

    delay(TIEMPO_AVANCE_20CM);

    parar();

    avanceDespuesDeGiro = false;

    // Pasar a la siguiente etapa:
    // 180+20 -> 360 -> 180+20 -> 180+20 -> 360 -> repetir.
    etapaGiroBusqueda++;

    if (etapaGiroBusqueda > 4)
      etapaGiroBusqueda = 0;

    Serial.println(">>> 20 CM COMPLETADOS <<<");
  }
}


// =====================================================
// ATACAR / EMPUJAR OBJETO
// =====================================================

void atacarObjeto()
{
  int distancia1 = distanciaLaser1();
  int distancia2 = distanciaLaser2();


  Serial.print("ATAQUE - Laser 1: ");
  Serial.print(distancia1);
  Serial.print(" mm | Laser 2: ");
  Serial.print(distancia2);
  Serial.println(" mm");


  // ===================================================
  // PRIORIDAD ABSOLUTA: BORDE DURANTE ATAQUE
  // ===================================================
  //
  // Se comprueba ANTES de los laser y del movimiento de
  // ataque. Con solo 3 sensores blancos se abandona el
  // empuje y se inicia el escape inmediatamente.

  if (detectaBordeDuranteAtaque())
  {
    Serial.println(">>> 3+ SENSORES EN BORDE DURANTE ATAQUE <<<");
    Serial.println(">>> ESCAPE INMEDIATO <<<");

    parar();
    estado = ESCAPAR_BORDE;

    return;
  }


  // ===================================================
  // ESTADO DE DETECCIÓN
  // ===================================================

  bool laser1Detecta =
    (distancia1 >= 45 &&
     distancia1 <= DISTANCIA_OBJETO);

  bool laser2Detecta =
    (distancia2 >= 45 &&
     distancia2 <= DISTANCIA_OBJETO);

  // Durante el ataque siempre se utiliza la distancia normal.
  distanciaDeteccionActual = DISTANCIA_OBJETO;


  // ===================================================
  // LOS DOS LASER DETECTAN
  // ===================================================
  //
  // Objeto probablemente centrado.
  // Ataque recto.
  // ===================================================

  if (laser1Detecta && laser2Detecta)
  {
    Serial.println(">>> OBJETO CENTRADO <<<");
    Serial.println(">>> ATAQUE RECTO <<<");

    MiniR4.DriveDC.Move(
      VELOCIDAD_ATAQUE,
      VELOCIDAD_ATAQUE
    );

    return;
  }


  // ===================================================
  // SOLO LASER 1
  // ===================================================
  //
  // Objeto hacia el lado del Laser 1.
  // Se realiza una pequeña corrección.
  // ===================================================

  if (laser1Detecta && !laser2Detecta)
  {
    Serial.println(">>> OBJETO DETECTADO POR LASER 1 <<<");
    Serial.println(">>> CORRECCION HACIA LASER 1 <<<");

    MiniR4.DriveDC.Move(
      VELOCIDAD_ATAQUE - 10,
      VELOCIDAD_ATAQUE
    );

    return;
  }


  // ===================================================
  // SOLO LASER 2
  // ===================================================
  //
  // Objeto hacia el lado del Laser 2.
  // Se realiza una pequeña corrección.
  // ===================================================

  if (!laser1Detecta && laser2Detecta)
  {
    Serial.println(">>> OBJETO DETECTADO POR LASER 2 <<<");
    Serial.println(">>> CORRECCION HACIA LASER 2 <<<");

    MiniR4.DriveDC.Move(
      VELOCIDAD_ATAQUE,
      VELOCIDAD_ATAQUE - 10
    );

    return;
  }


  // ===================================================
  // NINGÚN LASER DETECTA EL OBJETO
  // ===================================================

  if (
    distancia1 > OBJETO_ALEJADO &&
    distancia2 > OBJETO_ALEJADO
  )
  {
    Serial.println(">>> OBJETO SACADO <<<");

    parar();

    delay(80);

    // Volver a buscar otro objeto
    iniciarBusqueda();

    return;
  }
}


// =====================================================
// ESCAPAR DEL BORDE
// =====================================================
//
// BORDE = BLANCO
//
// El robot:
// 1. Retrocede
// 2. Gira
// 3. Regresa al interior negro
// 4. Continúa buscando objetos
// =====================================================

void escaparBorde()
{
  Serial.println();
  Serial.println("************************");
  Serial.println("   BORDE BLANCO");
  Serial.println("   ESCAPANDO...");
  Serial.println("************************");


  // ===================================================
  // RETROCEDER
  // ===================================================

  MiniR4.DriveDC.Move(
    -VELOCIDAD_ESCAPE,
    -VELOCIDAD_ESCAPE
  );

  delay(1000);


  parar();

  delay(20);


  // ===================================================
  // GIRAR HACIA EL INTERIOR
  // ===================================================

  MiniR4.DriveDC.Move(
    VELOCIDAD_ESCAPE,
    -VELOCIDAD_ESCAPE
  );

  delay(450);


  parar();

  delay(80);


  // ===================================================
  // VOLVER A BUSCAR
  // ===================================================

  Serial.println(">>> VOLVIENDO AL INTERIOR <<<");

  iniciarBusqueda();
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  MiniR4.begin();

  MiniR4.PWR.setBattCell(2);

  Serial.begin(9600);


  // ===================================================
  // MOTORES
  // ===================================================

  configurarMotores();


  // ===================================================
  // LINE TRACER
  // ===================================================

  MiniR4.I2C0.MXLineTracer.begin();

  MiniR4.I2C0.MXLineTracer.setThreshold(NEGRO);


  // ===================================================
  // SENSOR DE COLOR
  // ===================================================

  sensorColorOK =
    MiniR4.I2C4.MXColorV3.begin();

  if (sensorColorOK)
  {
    Serial.println("Color V3: OK");
  }
  else
  {
    Serial.println("ERROR: Color V3");
  }


  // ===================================================
  // LASER 1
  // ===================================================
  //
  // Puerto I2C1
  // ===================================================

  bool laser1OK =
    MiniR4.I2C1.MXLaserV2.begin();

  if (laser1OK)
  {
    Serial.println("Laser V2 #1 I2C1: OK");
  }
  else
  {
    Serial.println("ERROR: Laser V2 #1 I2C1");
  }


  // ===================================================
  // LASER 2
  // ===================================================
  //
  // Puerto I2C2
  // ===================================================

  bool laser2OK =
    MiniR4.I2C2.MXLaserV2.begin();

  if (laser2OK)
  {
    Serial.println("Laser V2 #2 I2C2: OK");
  }
  else
  {
    Serial.println("ERROR: Laser V2 #2 I2C2");
  }


  errorAnterior = 0;


  // ===================================================
  // MODO INICIAL
  // ===================================================

  modoDojo = false;


  // ===================================================
  // BOTÓN UP
  // ===================================================

  Serial.println("Esperando boton UP...");

  while (!MiniR4.BTN_UP.getState())
  {
    delay(10);
  }

  delay(500);


  Serial.println();
  Serial.println("==============================");
  Serial.println("       ROBOT INICIADO");
  Serial.println("==============================");
  Serial.println("Laser 1: I2C1");
  Serial.println("Laser 2: I2C2");
  Serial.println("Distancia normal: 450 mm");
  Serial.println("Tras 1.5 s de busqueda: 1000 mm (1 metro)");
  Serial.println("PID rapido: Kp 30 / Kd 20");
  Serial.println("Secuencia: 180+20cm -> 360 -> 180+20cm -> 180+20cm -> 360");
}


// =====================================================
// LOOP PRINCIPAL
// =====================================================

void loop()
{

  // ===================================================
  // MODO 1: FAST LINE
  // ===================================================

  if (estado == SEGUIR_LINEA)
  {

    // -------------------------------------------------
    // FIN DE PISTA
    // -------------------------------------------------

    if (todosNegros())
    {
      Serial.println(">>> FIN DE PISTA <<<");

      parar();

      while (true)
      {
        delay(100);
      }
    }


    // -------------------------------------------------
    // ENTRADA AL DOJO
    // -------------------------------------------------

    if (detectaAzul())
    {
      Serial.println(">>> ENTRADA AL DOJO <<<");

      parar();

      delay(100);


      // Activar modo dojo

      modoDojo = true;


      Serial.println(">>> MODO DOJO ACTIVADO <<<");
      Serial.println(">>> INTERIOR = NEGRO <<<");
      Serial.println(">>> BORDE = BLANCO <<<");
      Serial.println(">>> LASER 1 = I2C1 <<<");
      Serial.println(">>> LASER 2 = I2C2 <<<");
      Serial.println(">>> DETECCION = 450 MM <<<");


      estado = IR_CENTRO;

      return;
    }


    // -------------------------------------------------
    // SEGUIR LÍNEA
    // -------------------------------------------------

    fastLine();
  }


  // ===================================================
  // MODO 2: IR AL CENTRO
  // ===================================================

  else if (estado == IR_CENTRO)
  {
    irCentro();
  }


  // ===================================================
  // MODO 3: BUSCAR OBJETO
  // ===================================================

  else if (estado == BUSCAR_OBJETO)
  {
    buscarObjeto();
  }


  // ===================================================
  // MODO 4: ATACAR OBJETO
  // ===================================================

  else if (estado == ATACAR_OBJETO)
  {
    atacarObjeto();
  }


  // ===================================================
  // MODO 5: ESCAPAR DEL BORDE
  // ===================================================

  else if (estado == ESCAPAR_BORDE)
  {
    escaparBorde();
  }


  delay(5);
}
