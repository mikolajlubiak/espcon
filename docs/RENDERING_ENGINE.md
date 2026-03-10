# Software 3D Rendering Engine

espcon implements a complete real-time 3D rendering pipeline entirely in software on a 240 MHz microcontroller with no dedicated graphics hardware. This document is a deep dive into how it works.

## Overview: No GPU, No Problem

Modern rendering offloads geometry transformation and pixel fill to dedicated GPU hardware. espcon has neither — every step runs on the ESP32-S3's dual Xtensa LX7 CPU cores. At 240 MHz with the `-Ofast` compiler flag, the ESP32-S3 is fast enough for real-time 3D rendering of moderate meshes on a small display.

The pipeline is:

```
OBJ File (LittleFS)
       |
       v
  Triangle Array (heap)
       |
       v
  [Per triangle, per frame]
       |
       +---> Model Transform (mat4 multiply)
       |
       +---> Back-face Culling (cross + dot product)
       |
       +---> Lighting (dot product with light direction)
       |
       +---> View Transform (camera look-at matrix)
       |
       +---> Perspective Projection (frustum matrix)
       |
       +---> Perspective Division (divide by w)
       |
       +---> Viewport Transform (NDC → pixels)
       |
       v
  Visible Triangle Buffer (trisToRaster[])
       |
       v
  Depth Sort (qsort, painter's algorithm)
       |
       v
  Rasterization (Adafruit fillTriangle → SPI)
       |
       v
  Display
```

---

## OBJ File Format and Parsing

### The Format

Wavefront OBJ is a simple ASCII mesh format. espcon uses only two line types:

```
v  1.000000 -1.000000 -1.000000    # vertex: x y z
f  1 5 6                           # face: v1 v2 v3 (1-indexed)
```

Lines starting with `#` (comments), `vt` (texture coords), `vn` (normals), `o` (object name), `s` (smoothing), etc., are silently ignored because the parser only branches on `line[0] == 'v'` and `line[0] == 'f'`.

**Note**: Only triangulated faces are supported. Export from Blender with "Triangulate Faces" enabled.

### The Parser

The parser reads from LittleFS one character at a time to avoid allocating a full file buffer (important on a microcontroller with limited RAM):

```
                         ┌─────────────────────────────────────┐
   file.read() ──► char  │  line buffer (dynamic, doubles on   │
                         │  overflow via realloc)               │
                    '\n' │                                       │
                     │   └─────────────────────────────────────┘
                     ▼
              line[0] == 'v' ?
                  │   yes ──► sscanf(line+2, "%f %f %f", &x, &y, &z)
                  │           push to verts[] (doubles on overflow)
                  │
              line[0] == 'f' ?
                  │   yes ──► sscanf(line+2, "%d %d %d", &i, &j, &k)
                  │           tris[n] = { verts[i-1], verts[j-1], verts[k-1] }
                  │           push to tris[] (doubles on overflow)
                  │
              memset line buffer, repeat
```

Both the vertex array and triangle array start at a pre-allocated size and double via `realloc` if they overflow:

```cpp
if (numFilledVerts > numAllocVerts) {
    numAllocVerts *= 2;
    verts = realloc(verts, numAllocVerts * sizeof(vec3));
}
```

This means the parser can load large meshes as long as there is sufficient heap. On the ESP32-S3-N16R8V with 8 MB PSRAM, this is a very generous limit, though rendering performance degrades linearly with triangle count.

---

## 3D Math on a Microcontroller

### Coordinate Systems

The engine uses a right-handed coordinate system, consistent with OpenGL conventions:

```
        +Y
        |
        |   +Z (out of screen, towards viewer)
        |  /
        | /
        +--------+X
```

Vertices are stored in object space (the local coordinate frame of the mesh as exported from the 3D editor). The rendering pipeline transforms them through world space, view space, and finally NDC/screen space.

### Matrix Layout

Matrices are stored in row-major order as `float m[4][4]`. The matrix-vector multiply is a row-vector × matrix product:

```
v' = v * M

v'.x = v.x*M[0][0] + v.y*M[1][0] + v.z*M[2][0] + v.w*M[3][0]
v'.y = v.x*M[0][1] + v.y*M[1][1] + v.z*M[2][1] + v.w*M[3][1]
v'.z = v.x*M[0][2] + v.y*M[1][2] + v.z*M[2][2] + v.w*M[3][2]
v'.w = v.x*M[0][3] + v.y*M[1][3] + v.z*M[2][3] + v.w*M[3][3]
```

All transformations (model, view, projection) are composed as right-multiplications of the vertex by the matrix.

### Perspective Projection Matrix

`perspective(fFov, fAspectRatio, fNear, fFar)` builds a frustum projection matrix:

```
fov = 1 / tan(fFov / 2)

[ aspectRatio * fov,    0,               0,             0 ]
[                 0,  fov,               0,             0 ]
[                 0,    0,  far/(far-near),             1 ]
[                 0,    0, -far*near/(far-near),        0 ]
```

- `m[0][0] = aspectRatio * fov` — scales x proportionally to the aspect ratio
- `m[1][1] = fov` — scales y by the field of view
- `m[2][2]`, `m[3][2]` — encodes depth into the z and w components
- `m[2][3] = 1` — copies z into w so that perspective division (divide by w) creates correct foreshortening

### Look-At (View) Matrix

`point_at(pos, target, up)` constructs the camera-to-world matrix. Inverting it gives the world-to-camera (view) matrix. In this implementation, `GetViewMatrix()` returns the `point_at` result directly and uses it as the view matrix (equivalent to the mathematical inverse for an orthonormal basis):

```
newRight = normalize(cross(Up, Target))
Up       = normalize(Up - Target * dot(up, Target))
Target   = normalize(target - pos)

M = [ right.x,  right.y,  right.z,  0 ]
    [ up.x,     up.y,     up.z,     0 ]
    [ fwd.x,    fwd.y,    fwd.z,    0 ]
    [ pos.x,    pos.y,    pos.z,    1 ]
```

### Camera: Euler Angles

The camera is controlled with Yaw (rotation around the world Y-axis) and Pitch (rotation around the camera's local X-axis). The front vector is recomputed from these angles every time the camera moves:

```
front.x = cos(yaw) * cos(pitch)
front.y = sin(pitch)
front.z = sin(yaw) * cos(pitch)
front   = normalize(front)
right   = normalize(cross(front, worldUp))
up      = normalize(cross(right, front))
```

Pitch is clamped to ±89° to prevent gimbal lock at the poles.

---

## Rasterization

### What "Rasterization" Means Here

Rasterization is the process of filling all pixels inside a triangle boundary on screen. espcon delegates this entirely to `Adafruit_GFX::fillTriangle(x0, y0, x1, y1, x2, y2, color)`. The Adafruit implementation uses scan-line conversion: for each horizontal row (scanline) between the triangle's top and bottom vertices, it computes the left and right x boundaries by linear interpolation along the triangle edges, then fills all pixels in that horizontal span.

This is the most computationally expensive part of the pipeline. The number of SPI bytes written is proportional to the triangle's screen area.

### Why No z-Buffer?

A z-buffer would require storing a depth value per pixel and testing it before each write — on a 240×240 display, that is 57,600 floats (225 KB), which exceeds the ESP32-S3's internal SRAM. Instead, the engine uses the painter's algorithm: triangles are sorted by their average z-depth and drawn back-to-front. Nearer triangles overwrite farther ones naturally.

The painter's algorithm is fast (one `qsort` call per frame) and works well for convex meshes. It can produce incorrect results for intersecting or concave geometry.

---

## Back-face Culling

For a closed mesh, roughly half of all triangles face away from the camera at any given moment. Back-face culling eliminates these before the expensive view/projection transforms and rasterization:

```
line1 = p[1] - p[0]             // two edges from vertex 0
line2 = p[2] - p[0]
normal = normalize(cross(line1, line2))  // surface normal
cameraRay = p[0] - cameraPosition       // vector from vertex to camera

if dot(normal, cameraRay) < 0:          // normal points towards camera
    → visible, keep
else:
    → back-facing, discard
```

The cross product of the two edges follows the right-hand rule: for a counter-clockwise wound triangle (as seen from the front), the normal points towards the viewer when the triangle is front-facing.

---

## Lighting Model

The lighting model is a simple Lambertian (diffuse) model with an ambient floor:

```
luminance = dot(lightDirection, surfaceNormal)   // in [0, 1]

r = clamp(max(luminance, ambient) * baseR * 4, 0, 31)
g = clamp(max(luminance, ambient) * baseG * 4, 0, 63)
b = clamp(max(luminance, ambient) * baseB * 4, 0, 31)

color16 = (r << 11) | (g << 5) | b    // RGB565 packing
```

The factor of 4 is an amplification constant to make the lighting more visually pronounced.

The base color itself cycles over time using three independent sinusoidal waves:

```cpp
col.r = sin(theta * 2.0f) * 31;   // red:   2 Hz
col.g = sin(theta * 0.7f) * 63;   // green: 0.7 Hz
col.b = sin(theta * 1.3f) * 31;   // blue:  1.3 Hz
```

where `theta = elapsedTime / 1000` (seconds). This produces an animated, slowly-shifting color palette on the mesh surface.

---

## Frame Buffer Management

There is no off-screen frame buffer. The display is cleared to black at the start of each frame (`fillScreen(BLACK)`), then triangles are drawn directly to the display by issuing SPI commands. This is a **single-buffered** approach:

```
Frame start:
  tft.fillScreen(BLACK)     // SPI: clear all pixels

For each visible triangle (sorted back-to-front):
  tft.fillTriangle(...)     // SPI: write pixels for this triangle

Frame end:
  delay(remaining_time)     // yield until next frame
```

**Tearing**: Because there is no vsync and no double buffer, the display may be partially updated while it is being scanned out. On small TFT displays driven over SPI, this is typically not perceptible.

**Memory cost**: Avoiding a frame buffer saves 57,600 bytes (at 16 bpp for 240×240) of precious RAM, which is instead available for mesh data and the rendering pipeline itself.

---

## Performance Considerations

| Factor | Impact | How espcon addresses it |
|--------|--------|------------------------|
| Vertex count | O(n) transforms per frame | Keep meshes simple; OBJ parser supports any size but rendering time grows linearly |
| Triangle area | Proportional SPI bytes | Larger triangles = more SPI writes; keep display small |
| SPI clock speed | Limits fill rate | ST7735: 79.999 MHz; ILI9341: library default (~40 MHz) |
| Back-face culling | Halves transform count | Enabled by default, discards ~50% of triangles for closed meshes |
| `-Ofast` flag | ~10–30% faster math | Enabled globally; allows aggressive FP optimizations |
| `float` vs `double` | Float is hardware-accelerated on ESP32-S3 | All math uses `float` |
| Depth sort | O(n log n) | qsort on the visible triangle buffer (not the full mesh) |
| Frame-rate cap | Prevents busy-loop | `delay()` if frame finishes early |

The limiting factor in practice is the SPI fill rate: every call to `fillTriangle()` issues many small SPI transactions. Batching these (e.g., with DMA) would significantly increase throughput but would require bypassing the Adafruit driver.
