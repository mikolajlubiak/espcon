#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

#include <cstdint>
#include <cmath>

#include "models.h"

#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8
#define HEIGHT 128
#define WIDTH 128
#define FPS 30
#define FRAME_DELAY 1000 / FPS

struct vec3d
{
    float x, y, z;
};

struct mat4
{
    float m[4][4] = {0.0f};
};

struct triangle
{
    vec3d p[3];
    uint16_t col;
};

struct mesh
{
    triangle *tris;
    uint32_t numTris;
};

mesh *allocMesh(uint32_t numTris)
{
    mesh *mMesh = reinterpret_cast<mesh *>(calloc(1, sizeof(mesh)));
    mMesh->tris = reinterpret_cast<triangle *>(calloc(numTris, sizeof(triangle)));
    mMesh->numTris = numTris;
    return mMesh;
}

void freeMesh(mesh *mMesh)
{
    // Check if the triangle array pointer is not nullptr before freeing it
    if (mMesh && mMesh->tris)
    {
        free(mMesh->tris);
        mMesh->tris = nullptr;
    }

    // Check if the mesh pointer itself is not nullptr before freeing it
    if (mMesh)
    {
        free(mMesh);
    }
}

mesh *initMeshCube()
{
    constexpr uint32_t numTris = 12;

    mesh *mMesh = reinterpret_cast<mesh *>(calloc(1, sizeof(mesh)));
    mMesh->tris = reinterpret_cast<triangle *>(calloc(numTris, sizeof(triangle)));
    mMesh->numTris = numTris;

    // Cube vertices
    const vec3d vertices[8] = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        {0.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {1.0f, 0.0f, 1.0f}};

    // Triangle indices
    int triIndices[numTris][3] = {
        // SOUTH
        {0, 6, 4},
        {0, 4, 5},
        // EAST
        {5, 4, 3},
        {5, 3, 7},
        // NORTH
        {7, 3, 2},
        {7, 2, 1},
        // WEST
        {1, 2, 6},
        {1, 6, 0},
        // TOP
        {6, 2, 3},
        {6, 3, 4},
        // BOTTOM
        {7, 1, 0},
        {7, 0, 5}};

    // Assign vertices to triangles
    for (uint32_t i = 0; i < numTris; ++i)
    {
        mMesh->tris[i].p[0] = vertices[triIndices[i][0]];
        mMesh->tris[i].p[1] = vertices[triIndices[i][1]];
        mMesh->tris[i].p[2] = vertices[triIndices[i][2]];
    }

    return mMesh;
}

char *sgets(char *buffer, const uint32_t buffer_size, const unsigned char **strp, const uint32_t str_size)
{
    if ((**strp == '\0') | (reinterpret_cast<uint32_t>(strp) > reinterpret_cast<uint32_t>(*strp + str_size)))
        return nullptr;
    int i;
    for (i = 0; i < buffer_size - 1; ++i, ++(*strp))
    {
        buffer[i] = **strp;
        if (**strp == '\0')
            break;
        if (**strp == '\n')
        {
            buffer[i + 1] = '\0';
            ++(*strp);
            break;
        }
    }
    if (i == buffer_size - 1)
        buffer[i] = '\0';
    return buffer;
}

mesh *loadObj(const unsigned char *model, uint32_t len)
{
    mesh *mMesh = reinterpret_cast<mesh *>(calloc(1, sizeof(mesh)));

    mMesh->numTris = 512;
    mMesh->tris = reinterpret_cast<triangle *>(calloc(512, sizeof(triangle)));

    vec3d *verts = reinterpret_cast<vec3d *>(calloc(1024, sizeof(vec3d)));
    uint32_t numVerts = 0;

    char buff[128];
    const unsigned char **p = reinterpret_cast<const unsigned char **>(&model);

    while (NULL != sgets(buff, sizeof(buff), p, len))
    {
        Serial.println(buff);

        if (buff[0] == 'v')
        {
            sscanf(buff + 2, "%f %f %f", &verts[numVerts].x, &verts[numVerts].y, &verts[numVerts].z);
            numVerts++;
        }
        else if (buff[0] == 'f')
        {
            int f[3];
            sscanf(buff + 2, "%d %d %d", &f[0], &f[1], &f[2]);
            mMesh->tris[mMesh->numTris++] = {verts[f[0] - 1], verts[f[1] - 1], verts[f[2] - 1]};
        }
    }

    return mMesh;
}

void multiplyMatrixVector(const vec3d &i, vec3d &o, const mat4 &m)
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

int compareTris(const void *t1Ptr, const void *t2Ptr)
{
    const triangle *t1 = reinterpret_cast<const triangle *>(t1Ptr);
    const triangle *t2 = reinterpret_cast<const triangle *>(t2Ptr);
    float z1 = (t1->p[0].z + t1->p[1].z + t1->p[2].z) / 3;
    float z2 = (t2->p[0].z + t2->p[1].z + t2->p[2].z) / 3;
    return z1 > z2;
}

uint16_t getColor(float lum)
{
    int rb = max(lum, 0.20f) * 31;
    int g = max(lum, 0.20f) * 63;
    return ((rb << 11) | (g << 5) | rb);
}

class ESPCon
{
    Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

    mesh *mMesh = loadObj(ship_obj, ship_obj_len);

    triangle *trisToRaster = reinterpret_cast<triangle *>(calloc(mMesh->numTris, sizeof(triangle)));
    uint32_t numTrisToRaster = 0;

    mat4 matProj;

    vec3d vCamera = {0.0f};

    uint32_t deltaTime;
    uint32_t elapsedTime;
    float theta = 0.0f;

public:
    ESPCon()
    {
        Serial.begin(9600);
    }

    ~ESPCon()
    {
        freeMesh(mMesh);
        mMesh = nullptr;

        free(trisToRaster);
        trisToRaster = nullptr;
    }

    uint8_t setup()
    {
        tft.initR(INITR_144GREENTAB);

        // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
        // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
        // may end up with a black screen some times, or all the time.
        tft.setSPISpeed(79999999); // max tested on ESP32-S3 (80000000 produces artifacts)

        elapsedTime = millis();

        float fNear = 0.1f;
        float fFar = 1000.0f;
        float fFov = 90.0f;
        float fAspectRatio = static_cast<float>(HEIGHT) / WIDTH;
        float fFovRad = 1.0f / tan(fFov * 0.5f / 180.0f * 3.141592f);

        matProj.m[0][0] = fAspectRatio * fFovRad;
        matProj.m[1][1] = fFovRad;
        matProj.m[2][2] = fFar / (fFar - fNear);
        matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
        matProj.m[2][3] = 1.0f;
        matProj.m[3][3] = 0.0f;

        return 0;
    }

    void loop()
    {
        tft.fillScreen(ST77XX_BLACK);

        deltaTime = millis() - elapsedTime;
        elapsedTime = millis();
        theta = static_cast<float>(elapsedTime) / 1000;
        // Serial.println(1000.0f / deltaTime, DEC); // FPS

        mat4 matRotZ, matRotX;

        // Rotation Z
        matRotZ.m[0][0] = cos(theta);
        matRotZ.m[0][1] = sin(theta);
        matRotZ.m[1][0] = -sin(theta);
        matRotZ.m[1][1] = cos(theta);
        matRotZ.m[2][2] = 1;
        matRotZ.m[3][3] = 1;

        // Rotation X
        matRotX.m[0][0] = 1;
        matRotX.m[1][1] = cos(theta * 0.5f);
        matRotX.m[1][2] = sin(theta * 0.5f);
        matRotX.m[2][1] = -sin(theta * 0.5f);
        matRotX.m[2][2] = cos(theta * 0.5f);
        matRotX.m[3][3] = 1;

        numTrisToRaster = 0;

        // Draw triangles
        for (uint32_t i = 0; i < mMesh->numTris; i++)
        {
            triangle triProjected, triTranslated, triRotatedZ, triRotatedZX;

            // Rotate Z
            multiplyMatrixVector(mMesh->tris[i].p[0], triRotatedZ.p[0], matRotZ);
            multiplyMatrixVector(mMesh->tris[i].p[1], triRotatedZ.p[1], matRotZ);
            multiplyMatrixVector(mMesh->tris[i].p[2], triRotatedZ.p[2], matRotZ);

            // Rotate X
            multiplyMatrixVector(triRotatedZ.p[0], triRotatedZX.p[0], matRotX);
            multiplyMatrixVector(triRotatedZ.p[1], triRotatedZX.p[1], matRotX);
            multiplyMatrixVector(triRotatedZ.p[2], triRotatedZX.p[2], matRotX);

            // Translate triangles
            triTranslated = triRotatedZX;
            triTranslated.p[0].z = triTranslated.p[0].z + 8.0f;
            triTranslated.p[1].z = triTranslated.p[1].z + 8.0f;
            triTranslated.p[2].z = triTranslated.p[2].z + 8.0f;

            // Check if side is visible
            vec3d normal, line1, line2;

            line1.x = triTranslated.p[1].x - triTranslated.p[0].x;
            line1.y = triTranslated.p[1].y - triTranslated.p[0].y;
            line1.z = triTranslated.p[1].z - triTranslated.p[0].z;

            line2.x = triTranslated.p[2].x - triTranslated.p[0].x;
            line2.y = triTranslated.p[2].y - triTranslated.p[0].y;
            line2.z = triTranslated.p[2].z - triTranslated.p[0].z;

            normal.x = line1.y * line2.z - line1.z * line2.y;
            normal.y = line1.z * line2.x - line1.x * line2.z;
            normal.z = line1.x * line2.y - line1.y * line2.x;

            float normal_length = sqrt(pow(normal.x, 2) + pow(normal.y, 2) + pow(normal.z, 2));
            normal.x /= normal_length;
            normal.y /= normal_length;
            normal.z /= normal_length;

            if ((normal.x * (triTranslated.p[0].x - vCamera.x) +
                 normal.y * (triTranslated.p[0].y - vCamera.y) +
                 normal.z * (triTranslated.p[0].z - vCamera.z)) < 0.0f)
            {
                // Add basic lighting
                vec3d light_direction = {0.0f, 0.0f, -1.0f};

                float light_direction_length = sqrt(pow(light_direction.x, 2) + pow(light_direction.y, 2) +
                                                    pow(light_direction.z, 2));
                light_direction.x /= light_direction_length;
                light_direction.y /= light_direction_length;
                light_direction.z /= light_direction_length;

                float dp = normal.x * light_direction.x + normal.y * light_direction.y +
                           normal.z * light_direction.z;

                uint16_t c = getColor(dp);
                triProjected.col = c;

                multiplyMatrixVector(triTranslated.p[0], triProjected.p[0], matProj);
                multiplyMatrixVector(triTranslated.p[1], triProjected.p[1], matProj);
                multiplyMatrixVector(triTranslated.p[2], triProjected.p[2], matProj);

                // Scale into view
                triProjected.p[0].x += 1.0f;
                triProjected.p[0].y += 1.0f;
                triProjected.p[1].x += 1.0f;
                triProjected.p[1].y += 1.0f;
                triProjected.p[2].x += 1.0f;
                triProjected.p[2].y += 1.0f;

                triProjected.p[0].x *= 0.5f * static_cast<float>(WIDTH);
                triProjected.p[1].x *= 0.5f * static_cast<float>(WIDTH);
                triProjected.p[2].x *= 0.5f * static_cast<float>(WIDTH);
                triProjected.p[0].y *= 0.5f * static_cast<float>(HEIGHT);
                triProjected.p[1].y *= 0.5f * static_cast<float>(HEIGHT);
                triProjected.p[2].y *= 0.5f * static_cast<float>(HEIGHT);

                trisToRaster[numTrisToRaster++] = triProjected;
            }
        }

        // Sort triangles by depth (from back to front)
        qsort(trisToRaster, numTrisToRaster, sizeof(triangle), compareTris);

        for (uint32_t i = 0; i < numTrisToRaster; i++)
        {
            tft.fillTriangle(trisToRaster[i].p[0].x, trisToRaster[i].p[0].y, trisToRaster[i].p[1].x, trisToRaster[i].p[1].y, trisToRaster[i].p[2].x, trisToRaster[i].p[2].y, trisToRaster[i].col);
        }

        if (FRAME_DELAY > deltaTime)
        {
            delay(FRAME_DELAY - deltaTime);
        }
    }
};
