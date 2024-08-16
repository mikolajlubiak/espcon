#include <SPI.h>
#include <LittleFS.h>

#include <Adafruit_GFX.h>
#ifdef BUILD_ST7735
#include <Adafruit_ST7735.h>
#elif BUILD_ILI9341
#include <Adafruit_ILI9341.h>
#endif

#include "custom_math.h"

#include "camera.h"

constexpr uint8_t tftCS = 10;
constexpr uint8_t tftDC = 9;
constexpr uint8_t tftRST = 8;

constexpr uint8_t buttonsNum = 4;
constexpr uint8_t buttons[buttonsNum] = {1, 2, 4, 5};

constexpr uint16_t fps = 60;
constexpr float frame_delay = static_cast<float>(fps) / 1000;

struct color
{
    uint8_t r : 5;
    uint8_t g : 6;
    uint8_t b : 5;

    uint16_t load()
    {
        return ((r << 11) | (g << 5) | b);
    }

    uint16_t getColor(float lum)
    {
        uint8_t _r = max(lum, 0.2f) * this->r;
        uint8_t _g = max(lum, 0.2f) * this->g;
        uint8_t _b = max(lum, 0.2f) * this->b;
        return ((_r << 11) | (_g << 5) | _b);
    }
};

struct triangle
{
    vec4 p[3];
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
    if (mMesh == nullptr)
    {
        Serial.println("Error in allocMesh, mMesh == nullptr");
        return nullptr;
    }

    mMesh->tris = reinterpret_cast<triangle *>(calloc(numTris, sizeof(triangle)));
    if (mMesh->tris == nullptr)
    {
        Serial.println("Error in allocMesh, mMesh->tris == nullptr");
        return nullptr;
    }

    mMesh->numTris = numTris;
    return mMesh;
}

void freeMesh(mesh *mMesh)
{
    if (mMesh != nullptr)
    {
        if (mMesh->tris != nullptr)
        {
            free(mMesh->tris);
            mMesh->tris = nullptr;
        }

        free(mMesh);
    }
}

mesh *initMeshCube()
{
    constexpr uint32_t numTris = 12;

    mesh *mMesh = reinterpret_cast<mesh *>(calloc(1, sizeof(mesh)));
    if (mMesh == nullptr)
    {
        Serial.println("Error in initMeshCube, mMesh == nullptr");
        return nullptr;
    }

    mMesh->tris = reinterpret_cast<triangle *>(calloc(numTris, sizeof(triangle)));
    if (mMesh->tris == nullptr)
    {
        Serial.println("Error in initMeshCube, mMesh->tris == nullptr");
        return nullptr;
    }

    mMesh->numTris = numTris;

    // Cube vertices
    const vec4 vertices[8] = {
        {0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f, 1.0f},
        {0.0f, 1.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, 1.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 1.0f, 1.0f}};

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

mesh *loadObj(const char *path)
{
    constexpr uint32_t numLoadVerts = 512;
    constexpr uint32_t numLoadTris = 1024;
    constexpr uint32_t numLoadChars = 128;

    mesh *mMesh = reinterpret_cast<mesh *>(calloc(1, sizeof(mesh)));
    if (mMesh == nullptr)
    {
        Serial.println("Error in loadObj, mMesh == nullptr");
        return nullptr;
    }

    File file = LittleFS.open(path);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return nullptr;
    }

    mMesh->numTris = numLoadTris;
    mMesh->tris = reinterpret_cast<triangle *>(calloc(mMesh->numTris, sizeof(triangle)));
    if (mMesh->tris == nullptr)
    {
        Serial.println("Error in loadObj, mMesh->tris == nullptr");
        return nullptr;
    }

    uint32_t numFilledTris = 0;

    uint32_t numAllocVerts = numLoadVerts;
    vec3 *verts = reinterpret_cast<vec3 *>(calloc(numAllocVerts, sizeof(vec3)));
    if (verts == nullptr)
    {
        Serial.println("Error in loadObj, verts == nullptr");
        return nullptr;
    }

    uint32_t numFilledVerts = 0;

    uint32_t numAllocChars = numLoadChars;
    char *line = reinterpret_cast<char *>(calloc(numAllocChars, sizeof(char)));
    if (line == nullptr)
    {
        Serial.println("Error in loadObj, line == nullptr");
        return nullptr;
    }

    uint32_t numFilledChars = 0;
    bool isNewLine = true;

    while (file.available())
    {
        char c = file.read();

        if (c == '\n')
        {
            if (!isNewLine)
            {
                if (line[0] == 'v')
                {
                    sscanf(line + 2, "%f %f %f", &verts[numFilledVerts].x, &verts[numFilledVerts].y, &verts[numFilledVerts].z);
                    numFilledVerts++;
                    if (numFilledVerts > numAllocVerts)
                    {
                        numAllocVerts = numAllocVerts * 2;
                        verts = reinterpret_cast<vec3 *>(realloc(verts, numAllocVerts * sizeof(vec3)));
                        if (verts == nullptr)
                        {
                            Serial.println("Error in loadObj, verts == nullptr after realloc");
                            return nullptr;
                        }
                    }
                }
                else if (line[0] == 'f')
                {
                    int f[3];
                    sscanf(line + 2, "%d %d %d", &f[0], &f[1], &f[2]);
                    mMesh->tris[numFilledTris] = {vec4(verts[f[0] - 1], 1.0f), vec4(verts[f[1] - 1], 1.0f), vec4(verts[f[2] - 1], 1.0f)};
                    numFilledTris++;
                    if (numFilledTris > mMesh->numTris)
                    {
                        mMesh->numTris = mMesh->numTris * 2;
                        mMesh->tris = reinterpret_cast<triangle *>(realloc(mMesh->tris, mMesh->numTris * sizeof(triangle)));
                        if (mMesh->tris == nullptr)
                        {
                            Serial.println("Error in loadObj, mMesh->tris == nullptr after realloc");
                            return nullptr;
                        }
                    }
                }
                memset(line, 0, numAllocChars * sizeof(char));
                numFilledChars = 0;
            }
            isNewLine = true;
        }
        else
        {
            isNewLine = false;
            line[numFilledChars] = c;
            numFilledChars++;
            if (numFilledChars > numAllocChars)
            {
                numAllocChars = numAllocChars * 2;
                line = reinterpret_cast<char *>(realloc(line, numAllocChars * sizeof(char)));
                if (line == nullptr)
                {
                    Serial.println("Error in loadObj, line == nullptr after realloc");
                    return nullptr;
                }
            }
        }
    }

    file.close();

    free(verts);
    verts = nullptr;

    free(line);
    line = nullptr;

    return mMesh;
}

void listDir(const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = LittleFS.open(dirname);
    if (!root)
    {
        Serial.println("- failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");

            Serial.print(file.name());
            time_t t = file.getLastWrite();
            struct tm *tmstruct = localtime(&t);
            Serial.printf(
                "  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour,
                tmstruct->tm_min, tmstruct->tm_sec);

            if (levels)
            {
                listDir(file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");

            Serial.print(file.size());
            time_t t = file.getLastWrite();
            struct tm *tmstruct = localtime(&t);
            Serial.printf(
                "  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour,
                tmstruct->tm_min, tmstruct->tm_sec);
        }
        file = root.openNextFile();
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

class ESPCon
{
#ifdef BUILD_ST7735
    Adafruit_ST7735 tft = Adafruit_ST7735(tftCS, tftDC, tftRST);
#elif BUILD_ILI9341
    Adafruit_ILI9341 tft = Adafruit_ILI9341(tftCS, tftDC);
#endif

#ifdef BUILD_ST7735
    const uint16_t height = tft.height();
    const uint16_t width = tft.width();
#elif BUILD_ILI9341
    // Thats not a bug, thats actually a hacky fix
    // For some reason my ILI9341 display can only draw on 1:1 aspect ratio surface (it clipps the rest)
    const uint16_t height = tft.width();
    const uint16_t width = tft.width();
#endif

    const float fAspectRatio = static_cast<float>(height) / width;

    mesh *mMesh = nullptr;

    triangle *trisToRaster = nullptr;
    uint32_t numTrisToRaster = 0;

    mat4 model = mat4(1.0f);
    mat4 view{};
    mat4 projection{};

    Camera camera = Camera(vec3(0.0f, 0.0f, 10.0f));
    vec3 vCameraRay{};
    vec3 normal{}, line1{}, line2{};

    triangle triTransformed{}, triViewed{}, triProjected{};

    uint8_t buttonsState = 0;

    uint32_t deltaTime = 0;
    uint32_t elapsedTime = 0;
    uint32_t lastTime = 0;
    float theta = 0.0f;

    color col{};

public:
    ESPCon() {}

    ~ESPCon()
    {
        freeMesh(mMesh);
        mMesh = nullptr;

        free(trisToRaster);
        trisToRaster = nullptr;
    }

    uint8_t setup()
    {
        Serial.begin(9600);

        if (!LittleFS.begin(true))
        {
            Serial.println("An Error has occurred while mounting LittleFS");
            return 1;
        }

        for (const uint8_t button : buttons)
        {
            pinMode(button, INPUT_PULLDOWN);
        }

#ifdef BUILD_ST7735
        tft.initR(INITR_144GREENTAB);

        // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
        // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
        // may end up with a black screen some times, or all the time.
        tft.setSPISpeed(79999999); // max tested on ESP32-S3 (80000000 produces artifacts)
#elif BUILD_ILI9341
        tft.begin();
        tft.setRotation(3);
#endif

        elapsedTime = millis();

        mMesh = loadObj("/littlefs/ship.obj");
        if (mMesh == nullptr)
        {
            Serial.println("Error occured while loading OBJ");
            return 1;
        }

        trisToRaster = reinterpret_cast<triangle *>(calloc(mMesh->numTris, sizeof(triangle)));
        if (trisToRaster == nullptr)
        {
            Serial.println("Error occured while reserving memory for trisToRaster");
            return 1;
        }

        return 0;
    }

    void loop()
    {
#ifdef BUILD_ST7735
        tft.fillScreen(ST7735_BLACK);
#elif BUILD_ILI9341
        tft.fillScreen(ILI9341_BLACK);
        yield();
#endif

        lastTime = elapsedTime;
        elapsedTime = millis();
        deltaTime = elapsedTime - lastTime;
        theta = static_cast<float>(elapsedTime) / 1000;
        // Serial.println(1000.0f/deltaTime, DEC); // FPS

        buttonsState = 0;

        for (uint8_t i = 0; i < buttonsNum; i++)
        {
            buttonsState = buttonsState | (digitalRead(buttons[i]) << i);
        }

        if ((buttonsState >> 0) == HIGH)
        {
            camera.ProcessKeyboard(RIGHT, deltaTime);
        }
        if ((buttonsState >> 1) == HIGH)
        {
            camera.ProcessKeyboard(LEFT, deltaTime);
        }
        if ((buttonsState >> 2) == HIGH)
        {
            camera.ProcessMouseMovement(10.0f, 0);
        }
        if ((buttonsState >> 3) == HIGH)
        {
            camera.ProcessMouseMovement(-10.0f, 0);
        }

        col.r = sin(theta * 2.0f) * 31;
        col.g = sin(theta * 0.7f) * 63;
        col.b = sin(theta * 1.3f) * 31;

        memset(trisToRaster, 0, sizeof(triangle) * mMesh->numTris);
        numTrisToRaster = 0;

        // Draw triangles
        for (uint32_t i = 0; i < mMesh->numTris; i++)
        {
            // Translate triangles
            triTransformed.p[0] = mMesh->tris[i].p[0] * model;
            triTransformed.p[1] = mMesh->tris[i].p[1] * model;
            triTransformed.p[2] = mMesh->tris[i].p[2] * model;

            // Check if side is visible
            line1 = triTransformed.p[1] - triTransformed.p[0];
            line2 = triTransformed.p[2] - triTransformed.p[0];

            normal = cross(line1, line2);
            normal = normalize(normal);

            vCameraRay = vec3(triTransformed.p[0]) - camera.Position;

            if (dot(normal, vCameraRay) < 0.0f)
            {
                // Add basic lighting
                vec3 light_direction = {0.0f, 0.0f, -1.0f};
                light_direction = normalize(light_direction);

                float dp = dot(light_direction, normal);

                uint16_t c = col.getColor(dp);
                triProjected.col = c;

                projection = perspective(deg_to_rad(camera.Zoom), fAspectRatio, 0.1f, 100.0f);

                view = camera.GetViewMatrix();

                triViewed.p[0] = triTransformed.p[0] * view;
                triViewed.p[1] = triTransformed.p[1] * view;
                triViewed.p[2] = triTransformed.p[2] * view;

                triProjected.p[0] = triViewed.p[0] * projection;
                triProjected.p[1] = triViewed.p[1] * projection;
                triProjected.p[2] = triViewed.p[2] * projection;

                triProjected.p[0] = triProjected.p[0] / triProjected.p[0].w;
                triProjected.p[1] = triProjected.p[1] / triProjected.p[0].w;
                triProjected.p[2] = triProjected.p[2] / triProjected.p[0].w;

                // Scale into view
                vec3 vOffsetView = {1.0f, 1.0f, 0.0f};
                triProjected.p[0] = triProjected.p[0] + vOffsetView;
                triProjected.p[1] = triProjected.p[1] + vOffsetView;
                triProjected.p[2] = triProjected.p[2] + vOffsetView;

                triProjected.p[0].x *= 0.5f * static_cast<float>(width);
                triProjected.p[1].x *= 0.5f * static_cast<float>(width);
                triProjected.p[2].x *= 0.5f * static_cast<float>(width);
                triProjected.p[0].y *= 0.5f * static_cast<float>(height);
                triProjected.p[1].y *= 0.5f * static_cast<float>(height);
                triProjected.p[2].y *= 0.5f * static_cast<float>(height);

                trisToRaster[numTrisToRaster++] = triProjected;
            }
        }

        // Sort triangles by depth (from back to front)
        qsort(trisToRaster, numTrisToRaster, sizeof(triangle), compareTris);

        for (uint32_t i = 0; i < numTrisToRaster; i++)
        {
            tft.fillTriangle(trisToRaster[i].p[0].x, trisToRaster[i].p[0].y, trisToRaster[i].p[1].x, trisToRaster[i].p[1].y, trisToRaster[i].p[2].x, trisToRaster[i].p[2].y, trisToRaster[i].col);
        }

#ifdef BUILD_ILI9341
        yield();
#endif

        if (frame_delay > deltaTime)
        {
            delay(frame_delay - deltaTime);
        }
    }
};
