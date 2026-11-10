/*
  Tilt Maze - Waveshare ESP32-S3-LCD-1.28 (GC9A01 round LCD + QMI8658 IMU)

  - Random circular maze (concentric rings + radial walls), new one every game
  - Ball starts at a random point on the outer ring, goal is the centre
  - Tilt the board to roll the ball
  - BOOT button: short press = new maze, hold ~1 s = recalibrate "level"

  Library needed: "GFX Library for Arduino" (moononournation)
  Board: ESP32S3 Dev Module, USB CDC On Boot = Enabled, Flash 16MB, PSRAM = QSPI
*/

#include <Arduino_GFX_Library.h>
#include <Wire.h>

// Newer Arduino_GFX versions removed the short colour names, so define them here
#ifndef BLACK
#define BLACK  0x0000
#define WHITE  0xFFFF
#define RED    0xF800
#define GREEN  0x07E0
#define BLUE   0x001F
#define CYAN   0x07FF
#define YELLOW 0xFFE0
#endif

// ---------------- Pins ----------------
#define PIN_SCK   10
#define PIN_MOSI  11
#define PIN_CS     9
#define PIN_DC     8
#define PIN_RST   12
#define PIN_BL    40
#define PIN_SDA    6
#define PIN_SCL    7
#define PIN_BOOT   0

// ---------------- Tilt mapping (flip these if the ball rolls the wrong way) ----
#define SWAP_XY    0   // 1 = swap sensor X and Y
#define INVERT_X   1   // 1 = invert left/right
#define INVERT_Y   1   // 1 = invert up/down
#define DEBUG_TILT 0   // 1 = print sensor values on Serial

// ---------------- Game tuning ----------------
const float ACC_GAIN    = 700.0f;  // px/s^2 per g of tilt
const float FRICTION    = 2.5f;    // velocity damping per second
const float MAX_SPEED   = 150.0f;  // px/s
const float DEADZONE    = 0.03f;   // g
//const float BOUNCE      = 0.25f;   // fraction of speed kept when hitting a wall
const float BOUNCE      = 0.95f;   // fraction of speed kept when hitting a wall
const float BALL_COLL_R = 4.0f;    // collision radius (px)
const int   NSAMP       = 16;      // collision sample points around the ball
const int   NNORM       = 32;      // sample points used to estimate the wall direction
const float SLIDE_ASSIST = 1.25f;  // share of blocked motion redirected along the wall (0 = pure physics)
const float WALL_T      = 1.0f;    // wall half-thickness for collision (px)

// ---------------- Display ----------------
Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, PIN_RST, 0 /*rotation*/, true /*IPS*/);

// ---------------- Maze geometry ----------------
const int   CX = 120, CY = 120;
const float CENTER_R = 14.0f;              // radius of the central goal area
const float RING_W   = 17.0f;              // width of each ring
const int   RINGS    = 6;                  // rings, index 0 = innermost, 5 = outermost
const int   CNT[RINGS] = {6, 12, 12, 24, 24, 24};  // sectors per ring (equal or doubling)
const int   MAXC = 24;

inline float ringR(int j) { return CENTER_R + RING_W * j; }  // j = 0..RINGS (RINGS = outer edge)

// wallR[k][s]: wall at the starting angle of sector s in ring k (between s-1 and s)
// wallA[k][s]: wall on the inner (centre-facing) edge of sector s in ring k
bool wallR[RINGS][MAXC];
bool wallA[RINGS][MAXC];
bool visited[RINGS][MAXC];

struct Cell { uint8_t k, s; };
struct Nb   { uint8_t k, s, isArc, wk, ws; };
Nb nbBuf[5];   // scratch buffer for neighbour search (global: avoids IDE prototype issue)

// ---------------- Ball state ----------------
float bx, by, vx, vy;
int   drawX, drawY;
uint32_t startMs;
float sampX[NSAMP], sampY[NSAMP];
float normX[NNORM], normY[NNORM];

// ---------------- IMU (QMI8658) ----------------
#define QMI_WHO_AM_I 0x00
#define QMI_CTRL1    0x02
#define QMI_CTRL2    0x03
#define QMI_CTRL7    0x08
#define QMI_AX_L     0x35
#define QMI_RESET    0x60

uint8_t imuAddr = 0;
float ax0 = 0, ay0 = 0;

bool i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t reg, uint8_t *buf, int n) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)imuAddr, (int)n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  const uint8_t addrs[2] = {0x6B, 0x6A};
  bool found = false;
  for (int i = 0; i < 2 && !found; i++) {
    imuAddr = addrs[i];
    uint8_t id = 0;
    if (i2cRead(QMI_WHO_AM_I, &id, 1) && id == 0x05) found = true;
  }
  if (!found) return false;
  i2cWrite(QMI_RESET, 0xB0);   // soft reset
  delay(30);
  i2cWrite(QMI_CTRL1, 0x40);   // address auto-increment
  i2cWrite(QMI_CTRL2, 0x05);   // accel +-2g, 250 Hz
  i2cWrite(QMI_CTRL7, 0x01);   // enable accelerometer
  delay(20);
  return true;
}

bool readAccel(float &ax, float &ay, float &az) {
  uint8_t b[6];
  if (!i2cRead(QMI_AX_L, b, 6)) return false;
  int16_t rx = (int16_t)(b[0] | (b[1] << 8));
  int16_t ry = (int16_t)(b[2] | (b[3] << 8));
  int16_t rz = (int16_t)(b[4] | (b[5] << 8));
  ax = rx / 16384.0f;
  ay = ry / 16384.0f;
  az = rz / 16384.0f;
  return true;
}

void centerText(const char *txt, int y, int size, uint16_t color) {
  int w = strlen(txt) * 6 * size;
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(CX - w / 2, y);
  gfx->print(txt);
}

void calibrateLevel() {
  gfx->fillScreen(BLACK);
  centerText("Hold level", 100, 2, WHITE);
  delay(1500);
  float sx = 0, sy = 0, az;
  int n = 0;
  for (int i = 0; i < 50; i++) {
    float x, y;
    if (readAccel(x, y, az)) { sx += x; sy += y; n++; }
    delay(10);
  }
  if (n) { ax0 = sx / n; ay0 = sy / n; }
}

// ---------------- Maze generation (recursive backtracker) ----------------
int collectNeighbors(int k, int s) {
  int n = 0;
  int c = CNT[k];
  int ns;

  ns = (s + 1) % c;                                   // clockwise
  if (!visited[k][ns]) nbBuf[n++] = {(uint8_t)k, (uint8_t)ns, 0, (uint8_t)k, (uint8_t)ns};
  ns = (s + c - 1) % c;                               // counter-clockwise
  if (!visited[k][ns]) nbBuf[n++] = {(uint8_t)k, (uint8_t)ns, 0, (uint8_t)k, (uint8_t)s};

  if (k > 0) {                                        // towards the centre
    ns = (CNT[k - 1] == c) ? s : s / 2;
    if (!visited[k - 1][ns]) nbBuf[n++] = {(uint8_t)(k - 1), (uint8_t)ns, 1, (uint8_t)k, (uint8_t)s};
  }
  if (k < RINGS - 1) {                                // towards the outside
    if (CNT[k + 1] == c) {
      if (!visited[k + 1][s]) nbBuf[n++] = {(uint8_t)(k + 1), (uint8_t)s, 1, (uint8_t)(k + 1), (uint8_t)s};
    } else {
      int a = 2 * s, b = 2 * s + 1;
      if (!visited[k + 1][a]) nbBuf[n++] = {(uint8_t)(k + 1), (uint8_t)a, 1, (uint8_t)(k + 1), (uint8_t)a};
      if (!visited[k + 1][b]) nbBuf[n++] = {(uint8_t)(k + 1), (uint8_t)b, 1, (uint8_t)(k + 1), (uint8_t)b};
    }
  }
  return n;
}

void generateMaze() {
  memset(wallR, 1, sizeof(wallR));
  memset(wallA, 1, sizeof(wallA));
  memset(visited, 0, sizeof(visited));

  Cell stack[RINGS * MAXC];
  int sp = 0;
  uint8_t k0 = random(RINGS);
  uint8_t s0 = random(CNT[k0]);
  stack[sp++] = {k0, s0};
  visited[k0][s0] = true;

  while (sp > 0) {
    Cell cur = stack[sp - 1];
    int n = collectNeighbors(cur.k, cur.s);
    if (n == 0) { sp--; continue; }
    Nb pick = nbBuf[random(n)];
    if (pick.isArc) wallA[pick.wk][pick.ws] = false;
    else            wallR[pick.wk][pick.ws] = false;
    visited[pick.k][pick.s] = true;
    stack[sp++] = {pick.k, pick.s};
  }

  // open exactly one door into the centre
  wallA[0][random(CNT[0])] = false;
}

// ---------------- Drawing ----------------
const uint16_t WALL_COLOR = 0x07FF;  // cyan

void drawArcWall(float R, float th0, float th1) {
  int steps = max(2, (int)ceilf(R * (th1 - th0) / 3.0f));
  for (int pass = 0; pass < 2; pass++) {
    float rr = R + (pass ? 0.5f : -0.5f);
    int px = lroundf(CX + rr * cosf(th0));
    int py = lroundf(CY + rr * sinf(th0));
    for (int i = 1; i <= steps; i++) {
      float th = th0 + (th1 - th0) * i / steps;
      int nx = lroundf(CX + rr * cosf(th));
      int ny = lroundf(CY + rr * sinf(th));
      gfx->drawLine(px, py, nx, ny, WALL_COLOR);
      px = nx; py = ny;
    }
  }
}

void drawRadialWall(float r0, float r1, float th) {
  float c = cosf(th), s = sinf(th);
  r0 -= 1.0f; r1 += 1.0f;  // fill the corners
  for (int pass = 0; pass < 2; pass++) {
    float off = pass ? 0.5f : -0.5f;
    int x0 = lroundf(CX + r0 * c - off * s);
    int y0 = lroundf(CY + r0 * s + off * c);
    int x1 = lroundf(CX + r1 * c - off * s);
    int y1 = lroundf(CY + r1 * s + off * c);
    gfx->drawLine(x0, y0, x1, y1, WALL_COLOR);
  }
}

void drawGoal() {
  gfx->fillCircle(CX, CY, 6, GREEN);
}

void drawMaze() {
  gfx->fillScreen(BLACK);
  drawArcWall(ringR(RINGS), 0, TWO_PI);  // outer wall
  for (int k = 0; k < RINGS; k++) {
    int c = CNT[k];
    for (int s = 0; s < c; s++) {
      float th0 = s * TWO_PI / c, th1 = (s + 1) * TWO_PI / c;
      if (wallA[k][s]) drawArcWall(ringR(k), th0, th1);
      if (wallR[k][s]) drawRadialWall(ringR(k), ringR(k + 1), th0);
    }
  }
  drawGoal();
}

// ---------------- Collision ----------------
// Is the point (x, y) inside a wall (or outside the maze)?
bool inWall(float x, float y) {
  float dx = x - CX, dy = y - CY;
  float r = sqrtf(dx * dx + dy * dy);
  if (r > ringR(RINGS) - WALL_T) return true;     // outer wall
  if (r < CENTER_R - WALL_T) return false;        // inside the goal area

  float th = atan2f(dy, dx);
  if (th < 0) th += TWO_PI;

  for (int j = 0; j < RINGS; j++) {               // circular walls
    if (fabsf(r - ringR(j)) <= WALL_T) {
      int c = CNT[j];
      int s = (int)(th * c / TWO_PI) % c;
      if (wallA[j][s]) return true;
    }
  }
  for (int k = 0; k < RINGS; k++) {               // radial walls
    if (r >= ringR(k) - WALL_T && r <= ringR(k + 1) + WALL_T) {
      int c = CNT[k];
      float u = th * c / TWO_PI;
      float ur = roundf(u);
      int s = ((int)ur) % c;
      float archDist = fabsf(u - ur) * TWO_PI * r / c;
      if (archDist <= WALL_T && wallR[k][s]) return true;
    }
  }
  return false;
}

bool blocked(float x, float y) {
  if (inWall(x, y)) return true;
  for (int i = 0; i < NSAMP; i++) {
    if (inWall(x + sampX[i], y + sampY[i])) return true;
  }
  return false;
}

// Direction pointing away from the wall, estimated from the ball's blocked sample points
bool wallNormal(float x, float y, float *nx, float *ny) {
  float sxn = 0, syn = 0;
  for (int i = 0; i < NNORM; i++) {
    if (inWall(x + normX[i], y + normY[i])) { sxn -= normX[i]; syn -= normY[i]; }
  }
  float len = sqrtf(sxn * sxn + syn * syn);
  if (len < 0.01f) return false;
  *nx = sxn / len;
  *ny = syn / len;
  return true;
}

void moveBall(float dx, float dy) {
  int steps = (int)ceilf(max(fabsf(dx), fabsf(dy)));
  if (steps < 1) return;
  float sx = dx / steps, sy = dy / steps;
  for (int i = 0; i < steps; i++) {
    float tx = bx + sx, ty = by + sy;
    if (!blocked(tx, ty)) { bx = tx; by = ty; continue; }

    // Hit a wall: slide along it instead of stopping
    float wnx, wny;
    bool slid = false;
    if (wallNormal(tx, ty, &wnx, &wny)) {
      float d = sx * wnx + sy * wny;
      if (d < 0) {
        float tcx = -wny, tcy = wnx;                  // direction along the wall
        float tc = sx * tcx + sy * tcy;
        float px = sx - d * wnx, py = sy - d * wny;   // step minus the part pointing into the wall
        if (fabsf(tc) > 0.02f) {                      // give a little of the blocked motion back
          float extra = SLIDE_ASSIST * (-d) * (tc > 0 ? 1.0f : -1.0f);
          px += tcx * extra;
          py += tcy * extra;
        }
        // Curved walls make a straight slide touch the wall, so nudge away from it if needed
        for (int n = 0; n < 5 && !slid; n++) {
          float push = (n == 0) ? 0.0f : 0.12f * (float)(1 << (n - 1));   // 0, .12, .24, .48, .96 px
          float qx = bx + px + wnx * push, qy = by + py + wny * push;
          if (!blocked(qx, qy)) { bx = qx; by = qy; slid = true; }
        }
      }
      float vd = vx * wnx + vy * wny;                 // same for the velocity
      if (vd < 0) {
        vx -= (1.0f + BOUNCE) * vd * wnx;
        vy -= (1.0f + BOUNCE) * vd * wny;
      }
    }
    if (!slid) {                                      // last resort: axis-by-axis
      if (!blocked(bx + sx, by)) bx += sx;
      if (!blocked(bx, by + sy)) by += sy;
    }
  }
}

// ---------------- Game flow ----------------
void drawBall() {
  gfx->fillCircle(drawX, drawY, 3, RED);
}

void eraseBall() {
  gfx->fillCircle(drawX, drawY, 3, BLACK);
  if (abs(drawX - CX) < 12 && abs(drawY - CY) < 12) drawGoal();
}

void newGame() {
  generateMaze();
  drawMaze();
  int s = random(CNT[RINGS - 1]);
  float th = (s + 0.5f) * TWO_PI / CNT[RINGS - 1];
  float r = (ringR(RINGS - 1) + ringR(RINGS)) * 0.5f;
  bx = CX + r * cosf(th);
  by = CY + r * sinf(th);
  vx = vy = 0;
  drawX = lroundf(bx);
  drawY = lroundf(by);
  drawBall();
  startMs = millis();
}

void showWin() {
  float secs = (millis() - startMs) / 1000.0f;
  char buf[24];
  snprintf(buf, sizeof(buf), "%.1f s", secs);
  gfx->fillScreen(BLACK);
  centerText("SOLVED!", 90, 3, YELLOW);
  centerText(buf, 130, 2, WHITE);
  delay(2500);
  newGame();
}

uint32_t lastUs = 0;
uint32_t pressStart = 0;
bool btnWasDown = false;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BL, OUTPUT);
  digitalWrite(PIN_BL, HIGH);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  if (!gfx->begin()) {
    Serial.println("Display init failed");
    while (true) delay(1000);
  }
  gfx->fillScreen(BLACK);

  for (int i = 0; i < NSAMP; i++) {
    float a = i * TWO_PI / NSAMP;
    sampX[i] = BALL_COLL_R * cosf(a);
    sampY[i] = BALL_COLL_R * sinf(a);
  }
  for (int i = 0; i < NNORM; i++) {
    float a = i * TWO_PI / NNORM;
    normX[i] = BALL_COLL_R * cosf(a);
    normY[i] = BALL_COLL_R * sinf(a);
  }

  if (!imuInit()) {
    centerText("IMU not found", 110, 2, RED);
    Serial.println("QMI8658 not found");
    while (true) delay(1000);
  }

  randomSeed(esp_random());
  calibrateLevel();
  newGame();
  lastUs = micros();
}

void loop() {
  // --- BOOT button: short = new maze, long = recalibrate ---
  bool down = (digitalRead(PIN_BOOT) == LOW);
  if (down && !btnWasDown) pressStart = millis();
  if (!down && btnWasDown) {
    if (millis() - pressStart > 1000) { calibrateLevel(); }
    newGame();
    lastUs = micros();
  }
  btnWasDown = down;

  // --- time step ---
  uint32_t now = micros();
  float dt = (now - lastUs) / 1e6f;
  lastUs = now;
  if (dt > 0.03f) dt = 0.03f;

  // --- tilt -> acceleration ---
  float ax, ay, az;
  if (readAccel(ax, ay, az)) {
    float gx = ax - ax0, gy = ay - ay0;
#if SWAP_XY
    float t = gx; gx = gy; gy = t;
#endif
#if INVERT_X
    gx = -gx;
#endif
#if INVERT_Y
    gy = -gy;
#endif
    if (fabsf(gx) < DEADZONE) gx = 0;
    if (fabsf(gy) < DEADZONE) gy = 0;
#if DEBUG_TILT
    Serial.printf("gx=%.2f gy=%.2f\n", gx, gy);
#endif
    vx += gx * ACC_GAIN * dt;
    vy += gy * ACC_GAIN * dt;
  }

  // --- friction + speed limit ---
  float damp = 1.0f - FRICTION * dt;
  if (damp < 0) damp = 0;
  vx *= damp; vy *= damp;
  float sp = sqrtf(vx * vx + vy * vy);
  if (sp > MAX_SPEED) { vx *= MAX_SPEED / sp; vy *= MAX_SPEED / sp; }

  // --- move with collision ---
  moveBall(vx * dt, vy * dt);

  // --- redraw only when the pixel position changed ---
  int nx = lroundf(bx), ny = lroundf(by);
  if (nx != drawX || ny != drawY) {
    eraseBall();
    drawX = nx; drawY = ny;
    drawBall();
  }

  // --- reached the centre? ---
  float dx = bx - CX, dy = by - CY;
  if (dx * dx + dy * dy < 9.0f * 9.0f) showWin();

  delay(4);
}
