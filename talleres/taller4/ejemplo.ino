enum Estado { eAvanzar = 0, eRetroceder, eGirar };

Estado estado;

uint16_t g_timeStampMs;

void comportamiento_init()
{
  //define estado inicial y ejecuta accion de la trancision inicial, segun ejercicio 
  estado = eAvanzar; 
  pid_set_speed(SPEED_NORMAL, SPEED_NORMAL);
  //... 
  
}

bool IsBumperLeftOn() { return bumper_left(); }
bool IsBumperRightOn() { return bumper_right(); }

void comportamiento_loop()
{
  // Proceso estados, chequeo condiciones y ejecuto transiciones
  switch (estado)
  {
      case eAvanzar: 
        {
          if (IsBumperLeftOn() || IsBumperRightOn())
          {
            pid_set_speed(-SPEED_NORMAL, -SPEED_NORMAL);
            g_timeStampMs = millis();
            estado = eRetroceder; 
          } 
        } break;
      case eRetroceder:
        {
      if ((millis() - g_timeStampMs) > 3000)
            {
                pid_set_speed(SPEED_NORMAL, -SPEED_NORMAL);
                g_timeStampMs = millis();
              estado = eGirar; 
            }              
        } break;
      case eGirar:
        {
      if ((millis() - g_timeStampMs) > 3000)
            {
                pid_set_speed(SPEED_NORMAL, SPEED_NORMAL);
              estado = eAvanzar; 
            }              
        } break;    
  }
  //...
}

