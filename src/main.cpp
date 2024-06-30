#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

#include <iostream>
#include <vector>
#include <cmath>

#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8
#define HEIGHT 128
#define WIDTH 128

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

struct vec3d
{
  float x, y, z;
};

struct triangle
{
  vec3d p[3];
};

struct mesh
{
  std::vector<triangle> tris;
};

struct mat4
{
  float m[4][4] = {0};
};

mesh meshCube;

mat4 matProj;

uint32_t deltaTime;
uint32_t elapsedTime;

uint16_t color;
uint8_t r;

void MultiplyMatrixVector(vec3d &i, vec3d &o, mat4 &m)
{
  o.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + m.m[3][0];
  o.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + m.m[3][1];
  o.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + m.m[3][2];
  float w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + m.m[3][3];

  if (w != 0.0f)
  {
    o.x = o.x / w;
    o.y = o.y / w;
    o.z = o.z / w;
  }
}

void setup()
{
  Serial.begin(9600);

  tft.initR(INITR_144GREENTAB);

  // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  // tft.setSPISpeed(40000000);

  elapsedTime = millis();

  meshCube.tris = {
      // SOUTH
      {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f},

      // EAST
      {1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f},
      {1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f},

      // NORTH
      {1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f},
      {1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f},

      // WEST
      {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f},
      {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},

      // TOP
      {0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
      {0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f},

      // BOTTOM
      {1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f},
      {1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},

  };

  float fNear = 0.1f;
  float fFar = 1000.0f;
  float fFov = 90.0f;
  float fAspectRatio = (float)HEIGHT / WIDTH;
  float fFovRad = 1.0f / tanf(fFov * 0.5f / 180.0f * 3.141592f);

  matProj.m[0][0] = fAspectRatio * fFovRad;
  matProj.m[1][1] = fFovRad;
  matProj.m[2][2] = fFar / (fFar - fNear);
  matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
  matProj.m[2][3] = 1.0f;
  matProj.m[3][3] = 0.0f;
}

void loop()
{
  tft.fillScreen(ST77XX_BLACK);

  deltaTime = millis() - elapsedTime;
  elapsedTime = millis();
  // Serial.println(1000.0f/deltaTime, DEC); // FPS

  r = (r + 1) % 32;
  color = ((r << 11) | (0 << 5) | 31 - r);

  mat4 matRotZ, matRotX;

  float theta = (float)elapsedTime / 1000;

  // Rotation Z
  matRotZ.m[0][0] = cosf(theta);
  matRotZ.m[0][1] = sinf(theta);
  matRotZ.m[1][0] = -sinf(theta);
  matRotZ.m[1][1] = cosf(theta);
  matRotZ.m[2][2] = 1;
  matRotZ.m[3][3] = 1;

  // Rotation X
  matRotX.m[0][0] = 1;
  matRotX.m[1][1] = cosf(theta * 0.5f);
  matRotX.m[1][2] = sinf(theta * 0.5f);
  matRotX.m[2][1] = -sinf(theta * 0.5f);
  matRotX.m[2][2] = cosf(theta * 0.5f);
  matRotX.m[3][3] = 1;

  // Draw triangles
  for (triangle tri : meshCube.tris)
  {
    triangle triProjected, triTranslated, triRotatedZ, triRotatedZX;

    // Rotate triangles

    // Rotate Z
    MultiplyMatrixVector(tri.p[0], triRotatedZ.p[0], matRotZ);
    MultiplyMatrixVector(tri.p[1], triRotatedZ.p[1], matRotZ);
    MultiplyMatrixVector(tri.p[2], triRotatedZ.p[2], matRotZ);

    // Rotate X
    MultiplyMatrixVector(triRotatedZ.p[0], triRotatedZX.p[0], matRotX);
    MultiplyMatrixVector(triRotatedZ.p[1], triRotatedZX.p[1], matRotX);
    MultiplyMatrixVector(triRotatedZ.p[2], triRotatedZX.p[2], matRotX);

    // Translate triangles
    triTranslated = triRotatedZX;
    triTranslated.p[0].z = triTranslated.p[0].z + 3.0f;
    triTranslated.p[1].z = triTranslated.p[1].z + 3.0f;
    triTranslated.p[2].z = triTranslated.p[2].z + 3.0f;

    MultiplyMatrixVector(triTranslated.p[0], triProjected.p[0], matProj);
    MultiplyMatrixVector(triTranslated.p[1], triProjected.p[1], matProj);
    MultiplyMatrixVector(triTranslated.p[2], triProjected.p[2], matProj);

    // Scale into view
    triProjected.p[0].x += 1.0f;
    triProjected.p[0].y += 1.0f;
    triProjected.p[1].x += 1.0f;
    triProjected.p[1].y += 1.0f;
    triProjected.p[2].x += 1.0f;
    triProjected.p[2].y += 1.0f;

    triProjected.p[0].x *= 0.5f * (float)WIDTH;
    triProjected.p[1].x *= 0.5f * (float)WIDTH;
    triProjected.p[2].x *= 0.5f * (float)WIDTH;
    triProjected.p[0].y *= 0.5f * (float)HEIGHT;
    triProjected.p[1].y *= 0.5f * (float)HEIGHT;
    triProjected.p[2].y *= 0.5f * (float)HEIGHT;

    tft.drawTriangle(triProjected.p[0].x, triProjected.p[0].y, triProjected.p[1].x, triProjected.p[1].y, triProjected.p[2].x, triProjected.p[2].y, color);
  }
}
