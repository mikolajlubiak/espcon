#include <SPI.h>
#include <LittleFS.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

constexpr uint8_t tftCS = 10;
constexpr uint8_t tftDC = 9;
constexpr uint8_t tftRST = 8;

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
};

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
    free(mMesh->tris);
    mMesh->tris = nullptr;

    free(mMesh);
}

mesh *initMeshCube()
{
    constexpr uint32_t numTris = 12;

    mesh *mMesh = reinterpret_cast<mesh *>(calloc(1, sizeof(mesh)));
    if (mMesh == nullptr)
    {
        return nullptr;
    }

    mMesh->tris = reinterpret_cast<triangle *>(calloc(numTris, sizeof(triangle)));
    if (mMesh->tris == nullptr)
    {
        return nullptr;
    }

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

mesh *loadObj(const char *path)
{
    constexpr uint32_t numLoadVerts = 512;
    constexpr uint32_t numLoadTris = 1024;
    constexpr uint32_t numLoadChars = 128;

    mesh *mMesh = reinterpret_cast<mesh *>(calloc(1, sizeof(mesh)));

    File file = LittleFS.open(path);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return nullptr;
    }

    mMesh->numTris = numLoadTris;
    mMesh->tris = reinterpret_cast<triangle *>(calloc(mMesh->numTris, sizeof(triangle)));
    uint32_t numFilledTris = 0;

    uint32_t numAllocVerts = numLoadVerts;
    vec3d *verts = reinterpret_cast<vec3d *>(calloc(numAllocVerts, sizeof(vec3d)));
    uint32_t numFilledVerts = 0;

    uint32_t numAllocChars = numLoadChars;
    char *line = reinterpret_cast<char *>(calloc(numAllocChars, sizeof(char)));
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
                        verts = reinterpret_cast<vec3d *>(realloc(verts, numAllocVerts * sizeof(vec3d)));
                        if (verts == nullptr)
                        {
                            return nullptr;
                        }
                    }
                }
                else if (line[0] == 'f')
                {
                    int f[3];
                    sscanf(line + 2, "%d %d %d", &f[0], &f[1], &f[2]);
                    mMesh->tris[numFilledTris] = {verts[f[0] - 1], verts[f[1] - 1], verts[f[2] - 1]};
                    numFilledTris++;
                    if (numFilledTris > mMesh->numTris)
                    {
                        mMesh->numTris = mMesh->numTris * 2;
                        mMesh->tris = reinterpret_cast<triangle *>(realloc(mMesh->tris, mMesh->numTris * sizeof(triangle)));
                        if (mMesh->tris == nullptr)
                        {
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

uint16_t getColor(float lum, color col)
{
    int r = max(lum, 0.50f) * col.r;
    int g = max(lum, 0.50f) * col.g;
    int b = max(lum, 0.50f) * col.b;
    return ((r << 11) | (g << 5) | b);
}

class ESPCon
{
    Adafruit_ST7735 tft = Adafruit_ST7735(tftCS, tftDC, tftRST);

    const uint16_t height = tft.height();
    const uint16_t width = tft.width();

    mesh *mMesh;

    triangle *trisToRaster;
    uint32_t numTrisToRaster = 0;

    mat4 matProj;

    vec3d vCamera = {0.0f};

    uint32_t deltaTime;
    uint32_t elapsedTime;
    uint32_t lastTime;
    float theta;

    color col;

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

        tft.initR(INITR_144GREENTAB);

        // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
        // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
        // may end up with a black screen some times, or all the time.
        tft.setSPISpeed(79999999); // max tested on ESP32-S3 (80000000 produces artifacts)

        elapsedTime = millis();

        constexpr float fNear = 0.1f;
        constexpr float fFar = 1000.0f;
        constexpr float fFov = 90.0f;
        const float fAspectRatio = static_cast<float>(height) / width;
        const float fFovRad = 1.0f / tan(fFov * 0.5f / 180.0f * M_PI);

        matProj.m[0][0] = fAspectRatio * fFovRad;
        matProj.m[1][1] = fFovRad;
        matProj.m[2][2] = fFar / (fFar - fNear);
        matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
        matProj.m[2][3] = 1.0f;
        matProj.m[3][3] = 0.0f;

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
        tft.fillScreen(ST7735_BLACK);

        lastTime = elapsedTime;
        elapsedTime = millis();
        deltaTime = elapsedTime - lastTime;
        theta = static_cast<float>(elapsedTime) / 1000;
        // Serial.println(1000.0f/deltaTime, DEC); // FPS

        col.r = sin(theta * 2.0f) * 31;
        col.g = sin(theta * 0.7f) * 63;
        col.b = sin(theta * 1.3f) * 31;

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

        memset(trisToRaster, 0, sizeof(triangle) * mMesh->numTris);
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

            float l = sqrt(pow(normal.x, 2) + pow(normal.y, 2) + pow(normal.z, 2));
            normal.x /= l;
            normal.y /= l;
            normal.z /= l;

            if ((normal.x * (triTranslated.p[0].x - vCamera.x) +
                 normal.y * (triTranslated.p[0].y - vCamera.y) +
                 normal.z * (triTranslated.p[0].z - vCamera.z)) < 0.0f)
            {
                // Add basic lighting
                vec3d light_direction = {0.0f, 0.0f, -1.0f};

                float l = sqrt(pow(light_direction.x, 2) + pow(light_direction.y, 2) +
                               pow(light_direction.z, 2));
                light_direction.x /= l;
                light_direction.y /= l;
                light_direction.z /= l;

                float dp = normal.x * light_direction.x + normal.y * light_direction.y +
                           normal.z * light_direction.z;

                uint16_t c = getColor(dp, col);
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

        if (frame_delay > deltaTime)
        {
            delay(frame_delay - deltaTime);
        }
    }
};
