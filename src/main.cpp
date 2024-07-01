#include "espcon.h"

ESPCon engine;

void setup()
{
  uint8_t err = engine.setup();

  if (err != 0)
  {
    Serial.print("ERROR\n");
    exit(err);
  }
}

void loop()
{
  engine.loop();
}
