#include <arduinoBotLib.h>
#include <Multiplexer.h>
#include <Arduino.h>


#define NB_SENSOR 5
#define SENSOR_NO_LINE -1
#define SENSOR_CROSS_LEFT -2
#define SENSOR_CROSS_RIGHT -3
#define SENSOR_CROSS_T -4
//
//variables
const int VITESSE_MOY = 70;
const int VITESSE_TURN = 70;

int tabsensor[NB_SENSOR] = {0, 0, 0, 0, 0};
int seuil = 0;
int blanc = 0;
int valeurnumerique = 0;
int capteur = 2;
int capteur_avant = 2;
int compteur = 0;

//obj
ArduinoBot bot(tabsensor, TK3, TK4, TK2);

//
void testSharp()
{
  int val = bot.sharpReadDistance_mm();
  //
  Serial.println(val);
  delay(1000);
}
//
void testled()
{
  bot.setLed(true);
  delay(300);
  //
  bot.setLed(false);
  delay(300);
  //
}
//
void testMotor()
{

  bot.setMotorSpeeds(VITESSE_MOY, VITESSE_MOY, 2000);
  bot.setMotorStop(1000);

  bot.setMotorSpeeds(VITESSE_MOY, -VITESSE_MOY, 500);
  bot.setMotorStop(1000);

  bot.setMotorSpeeds(-VITESSE_MOY, VITESSE_MOY, 500);
  bot.setMotorStop(1000);

  bot.setMotorSpeeds(-VITESSE_MOY, -VITESSE_MOY, 2000);
  bot.setMotorStop(1000);
}
//
int moyenne(int a, int b)
{
  int tmp = a + b;
  tmp = tmp >> 1;
  return tmp;
}
//
void calibre()
{
  bot.sensorRead();
  //bot.sensorPrintToSerial();

  int tmp = 0;
  tmp = moyenne(tabsensor[0], tabsensor[4]);
  blanc = tmp;

  seuil = blanc >> 1;
}
//
int traitementCapteur()
{
  int tmp = 0;
  bot.sensorRead();
  //bot.sensorPrintToSerial();
  //
  for (int i = 0; i < NB_SENSOR; i++)
  {
    tmp = tabsensor[i];
    tabsensor[i] = blanc - tmp;
  }
  //
  for (int i = 0; i < NB_SENSOR; i++)
  {
    if (tabsensor[i] >= seuil)
    {
      tabsensor[i] = 1;
    }
    else
    {
      tabsensor[i] = 0;
    }
  }
  //
  //bot.sensorPrintToSerial();

  //valeur numerique
  valeurnumerique = tabsensor[NB_SENSOR - 1];
  for (int i = NB_SENSOR - 2; i >= 0; i--)
  {
    valeurnumerique = valeurnumerique << 1;
    valeurnumerique += tabsensor[i];
  }
  //case NO_LINE
  if (tabsensor[0] == 0 && tabsensor[1] == 0 && tabsensor[2] == 0 && tabsensor[3] == 0 && tabsensor[4] == 0)
  {
    //Serial.println("NO_LINE");
    return SENSOR_NO_LINE;
  }
  //case CROSS_LEFT
  if (tabsensor[0] == 0 && tabsensor[3] == 1 && tabsensor[4] == 1)
  {
    //Serial.println("CROSS_LEFT");
    return SENSOR_CROSS_LEFT;
  }
  //case CROSS_RIGHT
  if (tabsensor[0] == 1 && tabsensor[1] == 1 && tabsensor[4] == 0)
  {
    //Serial.println("CROSS_RIGHT");
    return SENSOR_CROSS_RIGHT;
  }
  //case CROSS_T
  if (tabsensor[0] == 1 && tabsensor[4] == 1)
  {
    //Serial.println("CROSS_T");
    return SENSOR_CROSS_T;
  }
  //cas normal
  if (valeurnumerique <= 7)
  {
    for (int i = 0; i < NB_SENSOR; i++)
    {
      if (tabsensor[i] == 1)
      {
        Serial.println(i);
        return i;
      }
    }
  }
  else
  {
    for (int i = NB_SENSOR - 1; i >= 0; i--)
    {
      if (tabsensor[i] == 1)
      {
        Serial.println(i);
        return i;
      }
    }
  }
  //defaul
  return 2;
}
//
void demiTour()
{
  bot.setMotorStop(10);
  bot.setLed(true);
  //
  bot.setMotorSpeeds(VITESSE_TURN, -VITESSE_TURN, 500);

  int capteur = -1;

  while (capteur < 2)
  {
    capteur = traitementCapteur();
    bot.setMotorSpeeds(VITESSE_TURN, -VITESSE_TURN, 50);
  }
  bot.setLed(false);
  bot.setMotorStop(10);
}
//
void suivrePiste(int capteur)
{
  static int err_n = 0;
  static int err_n_1 = 0;
  static int droite = 0;
  static int gauche = 0;
  int kp = 3;
  int kd = 3;

  //
  err_n = 20 - (10 * capteur);
  err_n = kp * err_n;

  droite = VITESSE_MOY - err_n;
  droite -= kd * (err_n - err_n_1);

  gauche = VITESSE_MOY + err_n;
  gauche += kd * (err_n - err_n_1);

  //sat
  if (droite > 100)
    droite = 100;
  if (gauche > 100)
    gauche = 100;
  //
  if (droite < -100)
    droite = -100;
  if (gauche < -100)
    gauche = -100;
  //mem
  err_n_1 = err_n;

  bot.setMotorSpeeds(gauche, droite);
}
//
void action()
{
  capteur = traitementCapteur();

  //obstacle ?
  int distance = bot.sharpReadDistance_mm();
  if (distance < 70)
  {
    demiTour();
    return;
  }
  //
  if (capteur >= 0)
  {
    suivrePiste(capteur);
    capteur_avant = capteur;
  }
  else
  {
    bot.setMotorStop(0);
    //
    switch (capteur)
    {
      case SENSOR_CROSS_LEFT:
        capteur = -1;
        //bot.setMotorSpeeds(VITESSE_TURN,VITESSE_TURN,100);
        while (capteur < 0 || capteur > 2)
        {
          bot.setMotorSpeeds(-VITESSE_TURN, VITESSE_TURN, 100);
          bot.setMotorStop(10);
          capteur = traitementCapteur();
        }
        return;
      //
      case SENSOR_CROSS_RIGHT:
        capteur = -1;
        //bot.setMotorSpeeds(VITESSE_TURN,VITESSE_TURN,100);
        while (capteur < 0 || capteur < 2)
        {
          bot.setMotorSpeeds(VITESSE_TURN, -VITESSE_TURN, 100);
          bot.setMotorStop(10);
          capteur = traitementCapteur();
        }
        return;
        ;

      case SENSOR_CROSS_T:
        capteur = -1;
        //bot.setMotorSpeeds(VITESSE_TURN,VITESSE_TURN,100);
        while (capteur < 0 || capteur < 2)
        {
          bot.setMotorSpeeds(-VITESSE_TURN, VITESSE_TURN, 100);
          bot.setMotorStop(10);
          capteur = traitementCapteur();
        }
        return;
        ;

      case SENSOR_NO_LINE:
        compteur++;
        if (compteur < 5)
        {
          if (capteur_avant <= 2)
          {
            bot.setMotorSpeeds(VITESSE_TURN, -VITESSE_TURN, 200);
          }
          else
          {
            bot.setMotorSpeeds(-VITESSE_TURN, VITESSE_TURN, 200);
          }
          //
          suivrePiste(capteur_avant);
        }
        else
        {
          compteur = 0;
          capteur = -1;
          while (capteur < 0 || capteur < 2)
          {
            bot.setMotorSpeeds(VITESSE_TURN, -VITESSE_TURN, 100);
            bot.setMotorStop(10);
            capteur = traitementCapteur();
          }
        }
        return;

      default:
        break;
    }
  }
}
//**************
void setup()
{
  Serial.begin(9600);
  //
  bot.begin();
  //
  calibre();
  bot.buzzer(440, 200);
  //
  delay(2000);
}
//**************
void loop()
{
  action();
}
