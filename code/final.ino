#include "MatrixMiniR4.h"

// =====================================================
// ROBOT / VERSION: DOJO 6 V3 - PID RAPIDO CON RETORNO A LINEA
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
float Kp = 24.0;
float Kd = 27.0;

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

const int DISTANCIA_OBJETO = 350;

// Distancia ampliada durante una búsqueda prolongada (55 cm).
const int DISTANCIA_BUSQUEDA_PROLONGADA = 1000;

// Si supera este valor, consideramos que el objeto ya no está delante.
const int OBJETO_ALEJADO = 350;


// =====================================================
// BÚSQUEDA PROLONGADA
// =====================================================

const unsigned long TIEMPO_BUSQUEDA_PROLONGADA = 1500;
unsigned long inicioBusqueda = 0;


// =====================================================
// AVANCE DESPUES DE LOS GIROS DE 180 GRADOS
// =====================================================

const unsigned long TIEMPO_GIRO_180 = 450;
const unsigned long TIEMPO_GIRO_360 = TIEMPO_GIRO_180 * 2;
const unsigned long TIEMPO_AVANCE_20CM = 360;

int etapaGiroBusqueda = 0;
bool avanceDespuesDeGiro = false;

bool busquedaProlongada = false;
int distanciaDeteccionActual = DISTANCIA_OBJETO;


// =====================================================
// VELOCIDADES DEL DOJO
// =====================================================

const int VELOCIDAD_CENTRO = 75;
const int VELOCIDAD_BUSQUEDA = 45;
const int VELOCIDAD_ATAQUE = 58;
const int VELOCIDAD_ESCAPE = 100;


// =====================================================
// TIEMPO PARA LLEGAR AL CENTRO
// =====================================================

const unsigned long TIEMPO_CENTRO = 900;


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

bool todosNegros()
{
  int sensoresNegros = 0;

  for (int i = 0; i < 10; i++)
  {
    float valor = MiniR4.I2C0.MXLineTracer.getSensor(i);

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

bool detectaBorde()
{
  int sensoresBorde = 0;

  for (int i = 0; i < 10; i++)
  {
    float valor = MiniR4.I2C0.MXLineTracer.getSensor(i);

    if (valor >= NEGRO)
    {
      sensoresBorde++;
    }
  }

  return sensoresBorde >= 1;
}


// =====================================================
// DETECTAR BORDE DURANTE ATAQUE
// =====================================================

bool detectaBordeDuranteAtaque()
{
  int sensoresBorde = 0;

  for (int i = 0; i < 10; i++)
  {
    float valor = MiniR4.I2C0.MXLineTracer.getSensor(i);

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
  float error = MiniR4.I2C0.MXLineTracer.getError();

  float derivada = error - errorAnterior;
  derivada = constrain(derivada, -25, 25);

  float correccion = (error * Kp) + (derivada * Kd);

  float velocidadActual = velocidad;
  if (abs(error) > 35)
    velocidadActual = 100;
  if (abs(error) > 52)
    velocidadActual = 93;

  float motorIzq = velocidadActual + correccion;
  float motorDer = velocidadActual - correccion;

  motorIzq = constrain(motorIzq, -100, 100);
  motorDer = constrain(motorDer, -100, 100);

  MiniR4.DriveDC.Move(motorIzq, motorDer);

  errorAnterior = error;
}


// =====================================================
// LECTURA DE LASERS
// =====================================================

int distanciaLaser1()
{
  int distancia = MiniR4.I2C1.MXLaserV2.getDistance();
  if (distancia >= 2000) return 2000;
  return distancia;
}

int distanciaLaser2()
{
  int distancia = MiniR4.I2C2.MXLaserV2.getDistance();
  if (distancia >= 2000) return 2000;
  return distancia;
}


// =====================================================
// DETECTAR OBJETO CON LOS DOS LASER
// =====================================================

bool objetoDetectado(int distancia1, int distancia2)
{
  bool laser1Detecta = (distancia1 >= 35 && distancia1 <= distanciaDeteccionActual);
  bool laser2Detecta = (distancia2 >= 35 && distancia2 <= distanciaDeteccionActual);

  return laser1Detecta || laser2Detecta;
}


// =====================================================
// INICIAR BÚSQUEDA
// =====================================================

void iniciarBusqueda()
{
  estado = BUSCAR_OBJETO;
  inicioBusqueda = millis();
  busquedaProlongada = false;
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
  Serial.println("Avanzando hacia el centro del dojo...");

  MiniR4.DriveDC.Move(VELOCIDAD_CENTRO, VELOCIDAD_CENTRO);
  delay(TIEMPO_CENTRO);

  parar();
  delay(100);

  modoDojo = true;
  iniciarBusqueda();
}


// =====================================================
// BUSCAR OBJETO
// =====================================================

void buscarObjeto()
{
  // REVISIÓN DE RETORNO AUTOMÁTICO A LÍNEA:
  // Si en cualquier momento de la búsqueda el sensor detecta el color azul, 
  // el robot sale del dojo y regresa a seguir la línea de la pista.
  if (detectaAzul())
  {
    Serial.println(">>> AZUL DETECTADO: REGRESANDO A SEGUIR LINEA <<<");
    parar();
    modoDojo = false;
    estado = SEGUIR_LINEA;
    return;
  }

  int distancia1 = distanciaLaser1();
  int distancia2 = distanciaLaser2();

  if (
    !busquedaProlongada &&
    (unsigned long)(millis() - inicioBusqueda) >= TIEMPO_BUSQUEDA_PROLONGADA
  )
  {
    busquedaProlongada = true;
    distanciaDeteccionActual = DISTANCIA_BUSQUEDA_PROLONGADA;
    Serial.println(">>> DETECCION AMPLIADA A 1 METRO <<<");
  }

  if (detectaBorde())
  {
    estado = ESCAPAR_BORDE;
    return;
  }

  if (objetoDetectado(distancia1, distancia2))
  {
    distanciaDeteccionActual = DISTANCIA_OBJETO;
    busquedaProlongada = false;
    estado = ATACAR_OBJETO;
    return;
  }

  if (!avanceDespuesDeGiro)
  {
    unsigned long tiempoGiro;
    int gradosGiro;

    if (etapaGiroBusqueda == 0 || etapaGiroBusqueda == 2 || etapaGiroBusqueda == 3)
    {
      tiempoGiro = TIEMPO_GIRO_180;
      gradosGiro = 180;
    }
    else
    {
      tiempoGiro = TIEMPO_GIRO_360;
      gradosGiro = 360;
    }

    MiniR4.DriveDC.Move(VELOCIDAD_BUSQUEDA, -VELOCIDAD_BUSQUEDA);

    unsigned long inicioGiro = millis();
    bool objetivoDuranteGiro = false;
    bool bordeDuranteGiro = false;

    while ((unsigned long)(millis() - inicioGiro) < tiempoGiro)
    {
      // Permitir detectar azul o borde también durante los giros
      if (detectaAzul())
      {
        parar();
        modoDojo = false;
        estado = SEGUIR_LINEA;
        return;
      }

      if (detectaBorde())
      {
        bordeDuranteGiro = true;
        break;
      }

      if (objetoDetectado(distanciaLaser1(), distanciaLaser2()))
      {
        objetivoDuranteGiro = true;
        break;
      }

      delay(1);
    }

    parar();

    if (bordeDuranteGiro)
    {
      estado = ESCAPAR_BORDE;
      return;
    }

    if (objetivoDuranteGiro)
    {
      distanciaDeteccionActual = DISTANCIA_OBJETO;
      busquedaProlongada = false;
      estado = ATACAR_OBJETO;
      return;
    }

    if (gradosGiro == 180)
    {
      avanceDespuesDeGiro = true;
    }
    else
    {
      avanceDespuesDeGiro = false;
      etapaGiroBusqueda++;
      if (etapaGiroBusqueda > 4) etapaGiroBusqueda = 0;
    }
  }
  else
  {
    MiniR4.DriveDC.Move(VELOCIDAD_BUSQUEDA, VELOCIDAD_BUSQUEDA);
    
    unsigned long inicioAvance = millis();
    while((unsigned long)(millis() - inicioAvance) < TIEMPO_AVANCE_20CM)
    {
      if (detectaAzul())
      {
        parar();
        modoDojo = false;
        estado = SEGUIR_LINEA;
        return;
      }
      if (detectaBorde())
      {
        estado = ESCAPAR_BORDE;
        return;
      }
      delay(1);
    }

    parar();
    avanceDespuesDeGiro = false;
    etapaGiroBusqueda++;
    if (etapaGiroBusqueda > 4) etapaGiroBusqueda = 0;
  }
}


// =====================================================
// ATACAR / EMPUJAR OBJETO
// =====================================================

void atacarObjeto()
{
  if (detectaAzul())
  {
    parar();
    modoDojo = false;
    estado = SEGUIR_LINEA;
    return;
  }

  int distancia1 = distanciaLaser1();
  int distancia2 = distanciaLaser2();

  if (detectaBordeDuranteAtaque())
  {
    parar();
    estado = ESCAPAR_BORDE;
    return;
  }

  bool laser1Detecta = (distancia1 >= 35 && distancia1 <= DISTANCIA_OBJETO);
  bool laser2Detecta = (distancia2 >= 35 && distancia2 <= DISTANCIA_OBJETO);
  distanciaDeteccionActual = DISTANCIA_OBJETO;

  if (laser1Detecta && laser2Detecta)
  {
    MiniR4.DriveDC.Move(VELOCIDAD_ATAQUE, VELOCIDAD_ATAQUE);
    return;
  }

  if (laser1Detecta && !laser2Detecta)
  {
    MiniR4.DriveDC.Move(VELOCIDAD_ATAQUE - 10, VELOCIDAD_ATAQUE);
    return;
  }

  if (!laser1Detecta && laser2Detecta)
  {
    MiniR4.DriveDC.Move(VELOCIDAD_ATAQUE, VELOCIDAD_ATAQUE - 10);
    return;
  }

  if (distancia1 > OBJETO_ALEJADO && distancia2 > OBJETO_ALEJADO)
  {
    parar();
    delay(80);
    iniciarBusqueda();
    return;
  }
}


// =====================================================
// ESCAPAR DEL BORDE
// =====================================================

void escaparBorde()
{
  MiniR4.DriveDC.Move(-VELOCIDAD_ESCAPE, -VELOCIDAD_ESCAPE);
  delay(700);

  parar();

  MiniR4.DriveDC.Move(VELOCIDAD_ESCAPE, -VELOCIDAD_ESCAPE);
  delay(350);

  parar();
  delay(80);

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

  configurarMotores();

  MiniR4.I2C0.MXLineTracer.begin();
  MiniR4.I2C0.MXLineTracer.setThreshold(NEGRO);

  sensorColorOK = MiniR4.I2C4.MXColorV3.begin();

  MiniR4.I2C1.MXLaserV2.begin();
  MiniR4.I2C2.MXLaserV2.begin();

  errorAnterior = 0;
  modoDojo = false;

  Serial.println("Esperando boton UP...");
  while (!MiniR4.BTN_UP.getState())
  {
    delay(10);
  }
  delay(500);

  Serial.println("Robot Iniciado.");
}


// =====================================================
// LOOP PRINCIPAL
// =====================================================

void loop()
{
  if (estado == SEGUIR_LINEA)
  {
    if (todosNegros())
    {
      parar();
      while (true) { delay(100); }
    }

    if (detectaAzul())
    {
      parar();
      delay(100);
      modoDojo = true;
      estado = IR_CENTRO;
      return;
    }

    fastLine();
  }
  else if (estado == IR_CENTRO)
  {
    irCentro();
  }
  else if (estado == BUSCAR_OBJETO)
  {
    buscarObjeto();
  }
  else if (estado == ATACAR_OBJETO)
  {
    atacarObjeto();
  }
  else if (estado == ESCAPAR_BORDE)
  {
    escaparBorde();
  }
}