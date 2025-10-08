#include <Wire.h>
#include <AS5600.h>
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
  #define SYS_VOL   3.3
#else
  #define SERIAL Serial
  #define SYS_VOL   5
#endif

#define CODEUR_PORTE A0



AMS_5600 ams5600;

int DELTA_POS_CAPT = 0;

void setup()
{
  SERIAL.begin(9600);
  Wire.begin();
  SERIAL.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
  while(1){
    if(ams5600.detectMagnet() == 1 ){
      SERIAL.print("Current Magnitude: ");
      SERIAL.print(ams5600.getMagnitude());
      uint8_t agc = ams5600.getAgc();
      if (agc < 150 && agc > 120) {
        SERIAL.print(". Gain du AGC correct : ");
        SERIAL.println(agc);
      } else {
        SERIAL.print(". Gain du AGC INcorrect : ");
        SERIAL.println(agc);
      }
      break;
    } else {
      SERIAL.println("Can not detect magnet");
    }
    delay(1000);
  }
  getMaxAngle();
}
/*******************************************************
/* Function: convertRawAngleToDegrees
/* In: angle data from AMS_5600::getRawAngle
/* Out: human readable degrees as float
/* Description: takes the raw angle and calculates
/* float value in degrees.
/*******************************************************/
float convertRawAngleToDegrees(word newAngle)
{
  return map((newAngle+DELTA_POS_CAPT)%4095,0,4095,0,360.0); /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
}

float getAnalogAngleToDegrees()
{
  return map((int)(analogRead(CODEUR_PORTE) * 5.0/3.3),0,1023,0,360.0); // problème d'échelle, corrigé si alimenté en 5V normalement
}

/* Fonction qui permet de récupérer l'angle de correction à introduire sur le capteur de position de la porte pour corriger l'alignement de l'aimant */
/* Demande d'ouvrir la porte. Enregistre la valeur maximale trouvée. Et retourne la valeur de correction à introduire. */

void getMaxAngle() {
  SERIAL.println("Ouvrir la porte complètement et maintenir (30s)");
  int init_time = millis();
  int time = 0;
  int ang = 0;
  while (time < 30000) {
    time = millis() - init_time;
    int angle = ams5600.getScaledAngle();
    if ((angle) > ang) {
      ang = angle;
    }
    if (ang - angle >= 4000) {
      ang = 0;
    }
    if (time % 1000 == 0) {
      SERIAL.print(time);
      SERIAL.print(",");
      SERIAL.print(ang);
      SERIAL.print(",");
      SERIAL.println(angle);
    }
  }
  SERIAL.print("Mesure de l'angle maximal du capteur : ");
  SERIAL.println(ang);
  DELTA_POS_CAPT = 4000 - ang;
}

void loop()
{
  SERIAL.print(convertRawAngleToDegrees(ams5600.getScaledAngle()));
  SERIAL.print(" Valeur de correction :");
  SERIAL.println(DELTA_POS_CAPT);
  delay(100);
}