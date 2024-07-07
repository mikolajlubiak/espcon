#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <LittleFS.h>

constexpr uint8_t tftCS = 10;
constexpr uint8_t tftRST = 9;
constexpr uint8_t tftDC = 8;

constexpr uint16_t height = 128;
constexpr uint16_t width = 128;
constexpr float aspectRatio = static_cast<float>(height) /
                              static_cast<float>(width);

constexpr uint16_t fps = 60;
constexpr float frame_delay = static_cast<float>(fps) / 1000;

struct color
{
    uint8_t r : 5;
    uint8_t g : 6;
    uint8_t b : 5;

    color() : r(0), g(0), b(0) {}
    color(const uint8_t r, const uint8_t g, const uint8_t b) : r(r), g(g), b(b) {}

    uint16_t load()
    {
        return ((r << 11) | (g << 5) | b);
    }
};

struct mat4
{
    float m[4][4] = {{0.0f}};

    void identity()
    {
        this->m[0][0] = 1.0f;
        this->m[1][1] = 1.0f;
        this->m[2][2] = 1.0f;
        this->m[3][3] = 1.0f;
    }

    void rotation_x(const float fAngleRad)
    {
        this->m[0][0] = 1.0f;
        this->m[3][3] = 1.0f;
        this->m[1][1] = cos(fAngleRad);
        this->m[2][2] = cos(fAngleRad);
        this->m[1][2] = sin(fAngleRad);
        this->m[2][1] = -sin(fAngleRad);
    }

    void rotation_y(const float fAngleRad)
    {
        this->m[1][1] = 1.0f;
        this->m[3][3] = 1.0f;
        this->m[0][0] = cos(fAngleRad);
        this->m[2][2] = cos(fAngleRad);
        this->m[0][2] = sin(fAngleRad);
        this->m[2][0] = -sin(fAngleRad);
    }

    void rotation_z(const float fAngleRad)
    {
        this->m[2][2] = 1.0f;
        this->m[3][3] = 1.0f;
        this->m[0][0] = cos(fAngleRad);
        this->m[1][1] = cos(fAngleRad);
        this->m[0][1] = sin(fAngleRad);
        this->m[1][0] = -sin(fAngleRad);
    }

    void translation(const float x, const float y, const float z)
    {
        this->m[0][0] = 1.0f;
        this->m[1][1] = 1.0f;
        this->m[2][2] = 1.0f;
        this->m[3][3] = 1.0f;
        this->m[3][0] = x;
        this->m[3][1] = y;
        this->m[3][2] = z;
    }

    void projection(const float fFovDegrees, const float fAspectRatio, const float fNear,
                    const float fFar)
    {
        float fFovRad = 1.0f / tan(fFovDegrees * 0.5f / 180.0f * 3.14159f);
        this->m[0][0] = fAspectRatio * fFovRad;
        this->m[1][1] = fFovRad;
        this->m[2][2] = fFar / (fFar - fNear);
        this->m[3][2] = (-fFar * fNear) / (fFar - fNear);
        this->m[2][3] = 1.0f;
        this->m[3][3] = 0.0f;
    }

    mat4 operator*=(const mat4 &other)
    {
        mat4 matrix;
        for (int c = 0; c < 4; c++)
            for (int r = 0; r < 4; r++)
                matrix.m[r][c] =
                    this->m[r][0] * other.m[0][c] + this->m[r][1] * other.m[1][c] +
                    this->m[r][2] * other.m[2][c] + this->m[r][3] * other.m[3][c];
        return matrix;
    }
};

mat4 operator*(const mat4 &lhs, const mat4 &rhs)
{
    mat4 matrix;
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            matrix.m[r][c] = lhs.m[r][0] * rhs.m[0][c] + lhs.m[r][1] * rhs.m[1][c] +
                             lhs.m[r][2] * rhs.m[2][c] + lhs.m[r][3] * rhs.m[3][c];
    return matrix;
}

struct vec3d
{
    float x, y, z, w;

    vec3d() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}

    vec3d(const float x, const float y, const float z) : x(x), y(y), z(z), w(1.0f) {}

    vec3d &operator+=(const vec3d &other)
    {
        this->x += other.x;
        this->y += other.y;
        this->z += other.z;
        return *this;
    }

    vec3d &operator-=(const vec3d &other)
    {
        this->x -= other.x;
        this->y -= other.y;
        this->z -= other.z;
        return *this;
    }

    vec3d &operator*=(const vec3d &other)
    {
        this->x *= other.x;
        this->y *= other.y;
        this->z *= other.z;
        return *this;
    }

    vec3d &operator*=(float k)
    {
        this->x *= k;
        this->y *= k;
        this->z *= k;
        return *this;
    }

    vec3d &operator/=(const vec3d &other)
    {
        this->x /= other.x;
        this->y /= other.y;
        this->z /= other.z;
        return *this;
    }

    vec3d &operator/=(float k)
    {
        this->x /= k;
        this->y /= k;
        this->z /= k;
        return *this;
    }

    float dot_product(const vec3d &other)
    {
        return this->x * other.x + this->y * other.y + this->z * other.z;
    }

    float length() { return sqrt(this->dot_product(*this)); }

    void normalize() { *this /= this->length(); }

    vec3d cross_product(const vec3d &other)
    {
        return {
            (this->y * other.z - this->z * other.y),
            (this->z * other.x - this->x * other.z),
            (this->x * other.y - this->y * other.x),
        };
    }

    vec3d operator*=(const mat4 &m)
    {
        vec3d v;
        v.x = this->x * m.m[0][0] + this->y * m.m[1][0] + this->z * m.m[2][0] +
              this->w * m.m[3][0];
        v.y = this->x * m.m[0][1] + this->y * m.m[1][1] + this->z * m.m[2][1] +
              this->w * m.m[3][1];
        v.z = this->x * m.m[0][2] + this->y * m.m[1][2] + this->z * m.m[2][2] +
              this->w * m.m[3][2];
        v.w = this->x * m.m[0][3] + this->y * m.m[1][3] + this->z * m.m[2][3] +
              this->w * m.m[3][3];
        return v;
    }
};

vec3d operator*(const vec3d &lhs, const mat4 &rhs)
{
    vec3d v;
    v.x = lhs.x * rhs.m[0][0] + lhs.y * rhs.m[1][0] + lhs.z * rhs.m[2][0] +
          lhs.w * rhs.m[3][0];
    v.y = lhs.x * rhs.m[0][1] + lhs.y * rhs.m[1][1] + lhs.z * rhs.m[2][1] +
          lhs.w * rhs.m[3][1];
    v.z = lhs.x * rhs.m[0][2] + lhs.y * rhs.m[1][2] + lhs.z * rhs.m[2][2] +
          lhs.w * rhs.m[3][2];
    v.w = lhs.x * rhs.m[0][3] + lhs.y * rhs.m[1][3] + lhs.z * rhs.m[2][3] +
          lhs.w * rhs.m[3][3];
    return v;
}

vec3d operator-(vec3d lhs, const vec3d &rhs)
{
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    lhs.z -= rhs.z;
    return lhs;
}

struct triangle
{
    vec3d p[3];
    uint16_t col;

    triangle() : p{vec3d(), vec3d(), vec3d()}, col(0) {}

    triangle(const vec3d &p1, const vec3d &p2, const vec3d &p3) : p{p1, p2, p3}, col(0) {}

    triangle &operator+=(const triangle &other)
    {
        this->p[0] += other.p[0];
        this->p[1] += other.p[1];
        this->p[2] += other.p[2];
        return *this;
    }

    triangle &operator-=(const triangle &other)
    {
        this->p[0] -= other.p[0];
        this->p[1] -= other.p[1];
        this->p[2] -= other.p[2];
        return *this;
    }

    triangle &operator*=(const triangle &other)
    {
        this->p[0] *= other.p[0];
        this->p[1] *= other.p[1];
        this->p[2] *= other.p[2];
        return *this;
    }

    triangle &operator/=(const triangle &other)
    {
        this->p[0] /= other.p[0];
        this->p[1] /= other.p[1];
        this->p[2] /= other.p[2];
        return *this;
    }
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
    // Check if the triangle array pointer is not NULL before freeing it
    if (mMesh && mMesh->tris)
    {
        free(mMesh->tris);
        mMesh->tris = nullptr;
    }

    // Check if the mesh pointer itself is not NULL before freeing it
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

uint16_t getColor(float lum, color col)
{
    int r = max(lum, 0.2f) * col.r;
    int g = max(lum, 0.2f) * col.g;
    int b = max(lum, 0.2f) * col.b;
    return ((r << 11) | (g << 5) | b);
}

class ESPCon
{
    Adafruit_ST7735 tft = Adafruit_ST7735(tftCS, tftDC, tftRST);

    mesh *mMesh;

    triangle *trisToRaster;
    uint32_t numTrisToRaster = 0;

    mat4 matProj{};

    vec3d vCamera{};

    uint32_t deltaTime;
    uint32_t elapsedTime;
    uint32_t lastTime;
    float theta;

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

        tft.initR(INITR_144GREENTAB);

        // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
        // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
        // may end up with a black screen some times, or all the time.
        tft.setSPISpeed(79999999); // max tested on ESP32-S3 (80000000 produces artifacts)

        elapsedTime = millis();

        matProj.projection(90.0f,
                           aspectRatio,
                           0.1f, 1000.0f);

        mMesh = loadObj("/littlefs/ship.obj");
        trisToRaster = reinterpret_cast<triangle *>(calloc(mMesh->numTris, sizeof(triangle)));

        return 0;
    }

    void loop()
    {
        tft.fillScreen(ST77XX_BLACK);

        lastTime = elapsedTime;
        elapsedTime = millis();
        deltaTime = elapsedTime - lastTime;
        theta = static_cast<float>(elapsedTime) / 1000;
        // Serial.println(1000.0f/deltaTime, DEC); // FPS

        col.r = sin(theta * 2.0f) * 31;
        col.g = sin(theta * 0.7f) * 63;
        col.b = sin(theta * 1.3f) * 31;

        mat4 matRotZ{}, matRotX{}, matTrans{};

        matRotZ.rotation_z(theta * 0.5f);
        matRotX.rotation_x(theta);
        matTrans.translation(0.0f, 0.0f, 16.0f);

        mat4 matWorld{};
        matWorld = matRotZ * matRotX;
        matWorld = matWorld * matTrans;

        memset(trisToRaster, 0, sizeof(triangle) * mMesh->numTris);
        numTrisToRaster = 0;

        // Draw triangles
        for (uint32_t i = 0; i < mMesh->numTris; i++)
        {
            triangle triProjected{}, triTransformed{};

            triTransformed.p[0] = mMesh->tris[i].p[0] * matWorld;
            triTransformed.p[1] = mMesh->tris[i].p[1] * matWorld;
            triTransformed.p[2] = mMesh->tris[i].p[2] * matWorld;

            // Check if side is visible
            vec3d normal{}, line1{}, line2{};

            // Get either side of triangle
            line1 = triTransformed.p[1] - triTransformed.p[0];
            line2 = triTransformed.p[2] - triTransformed.p[0];

            // Take cross product to get normal
            normal = line1.cross_product(line2);
            normal.normalize();

            // Get ray from triangle to camera
            vec3d vCameraRay = triTransformed.p[0] - vCamera;

            if (normal.dot_product(vCameraRay) < 0.0f)
            {
                // Set and normalize light direction
                vec3d light_direction = {0.0f, 0.0f, -1.0f};
                light_direction.normalize();

                // How alligned is triangle surface normal and light direction
                uint16_t c = getColor(light_direction.dot_product(normal), col);

                triProjected.col = c;

                triProjected.p[0] = triTransformed.p[0] * matProj;
                triProjected.p[1] = triTransformed.p[1] * matProj;
                triProjected.p[2] = triTransformed.p[2] * matProj;

                triProjected.p[0] /= triProjected.p[0].w;
                triProjected.p[1] /= triProjected.p[1].w;
                triProjected.p[2] /= triProjected.p[2].w;

                // Scale into view
                vec3d vOffsetView = {1.0f, 1.0f, 0.0f};
                triProjected.p[0] += vOffsetView;
                triProjected.p[1] += vOffsetView;
                triProjected.p[2] += vOffsetView;

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
