#include "espcon.h"

ESPCon engine{};

void setup()
{
  uint8_t err = engine.setup();
  if (err != 0)
  {
    Serial.println("ERROR");
    exit(err);
  }
}

void loop()
{
  engine.loop();
}
