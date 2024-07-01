#include "espcon.h"

ESPCon engine;

void setup()
{
  uint8_t err = engine.setup();

  if (err != 0)
  {
    Serial.printf("ERROR %uhh\n", err);
    exit(err);
  }
}

void loop()
{
  engine.loop();
}
