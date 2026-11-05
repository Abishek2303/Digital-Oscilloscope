/*
  ESP32 DSO - ST7789 240x320 (320x240 landscape)
  Libraries: Adafruit GFX, Adafruit BusIO, Adafruit ST7735 and ST7789
  Sampling: I2S built-in ADC1 on GPIO33 (original ESP32 only). Use arduino-esp32 core 2.0.17.

  TFT:  CS=GPIO5  DC=GPIO2  RST=GPIO4  SCK=GPIO18  MOSI=GPIO23  VCC/BLK=3V3  GND=GND
  Signal input: GPIO33 (0 - 3.3 V ONLY!)     Self-test: jumper GPIO25 -> GPIO33 (1 kHz square wave)

  OK = SET : running -> next menu item,  stopped -> next cursor
  PLUS / MINUS : change the selected item (hold to repeat) / move the cursor
  BACK = RUN / STOP
*/

#include <Arduino.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ---------- Display options ----------
#define TFT_CS          5
#define TFT_DC          2
#define TFT_RST         4
#define USE_SPI_MODE3   0      // 1 only if the screen stays blank
#define INVERT_MODE     2      // 2 = inversion OFF (your panel). 1 = ON if colours look like negatives. 0 = library default
#define ROTATION        1      // 3 if the picture is upside down

// ---------- Buttons ----------
// 1 = button between pin and 3V3 (active HIGH), pins 32 / 15 / 35 / 34
//     GPIO32 and 15 use an internal pull-down; GPIO35 and 34 NEED an external 10k to GND.
// 2 = no resistors: button between pin and GND (active LOW), pins 32 / 15 / 13 / 14
#define BUTTON_LAYOUT   1

#if BUTTON_LAYOUT == 1
  #define BTN_OK        32
  #define BTN_PLUS      15
  #define BTN_MINUS     35
  #define BTN_BACK      34
  #define BTN_ACTIVE_HIGH 1
#else
  #define BTN_OK        32
  #define BTN_PLUS      15
  #define BTN_MINUS     13
  #define BTN_BACK      14
  #define BTN_ACTIVE_HIGH 0
#endif
#define DEBOUNCE_MS      25
#define REPEAT_DELAY_MS  400
#define REPEAT_MS        120

// ---------- Other pins / options ----------
#define ADC_CH          ADC1_CHANNEL_5   // GPIO33
#define ENABLE_TEST_SIGNAL 1
#define TEST_PIN        25

// ---------- Layout ----------
#define SCR_W 320
#define SCR_H 240
#define STRIP_H 40
#define HDR_H 21                  // header bar height
#define BAR_Y 224                 // bottom bar start
#define PLOT_X 10
#define PLOT_Y 22
#define PLOT_W 300
#define PLOT_H 200
#define PX_DIV_X 30
#define PX_DIV_Y 25
#define PRE_N 30                  // pre-trigger samples (1 division)
#define SAMPLES_PER_DIV 30

// ---------- Capture ----------
#define TRACE_N   600
#define MAX_DECIM 10
#define HYST_MV   50

// ---------- Colours (RGB565) ----------
#define C_BLACK  0x0000
#define C_WHITE  0xFFFF
#define C_HDR    0x10A4           // dark blue-grey bars
#define C_GRID   0x2945           // dim grid dots
#define C_GRID2  0x4A69           // centre lines
#define C_FRAME  0x7BCF           // frame + ticks
#define C_TRACE  0xFFE0           // trace colour (yellow). 0x07E0 = green
#define C_YELLOW 0xFFE0
#define C_CYAN   0x07FF
#define C_GREEN  0x07E0
#define C_RED    0xF800
#define C_ORANGE 0xFD20
#define C_LGREY  0xBDF7

// ---------- Tables ----------
struct TimeBase { uint32_t rate; uint8_t decim; const char* label; };
const TimeBase TB[] = {
  {1000000, 1, "30us"},  {500000, 1, "60us"},  {200000, 1, "150us"},
  {100000,  1, "300us"}, {50000,  1, "600us"}, {20000,  1, "1.5ms"},
  {20000,   2, "3ms"},   {20000,  5, "7.5ms"}, {20000, 10, "15ms"}
};
const int TB_COUNT = sizeof(TB) / sizeof(TB[0]);
const float VDIV[4] = {0.1f, 0.2f, 0.5f, 1.0f};
const char* VLBL[4] = {"0.1V", "0.2V", "0.5V", "1V"};

enum { M_TIME, M_VDIV, M_OFFS, M_TRIG, M_EDGE, M_MODE, M_COUNT };

// ---------- Objects ----------
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 cv(SCR_W, STRIP_H);
int oy = 0;
esp_adc_cal_characteristics_t adcChars;

// ---------- State ----------
int  tb = 3;                    // timebase (default 300 us/div)
int  vi = 2;                    // volts/div (default 0.5 V)
int  offT = 0;                  // offset, 0.1 V units
int  trgU = 33;                 // trigger level, 0.05 V units (1.65 V)
bool trgRising = true;
int  trgMode = 0;               // 0 = AUTO, 1 = NORM, 2 = SINGLE
int  sel = M_TIME;
bool running = true;
bool triggered = false;
bool dataValid = false;
int  curSel = 0;                // stopped: 0 none, 1 V1, 2 V2, 3 T1, 4 T2
int  curV1 = PLOT_Y + 50, curV2 = PLOT_Y + 150, curT1 = PLOT_X + 60, curT2 = PLOT_X + 210;
uint32_t curRate = 0;

static uint16_t rawBuf[TRACE_N * MAX_DECIM];
static uint16_t tmpMv[TRACE_N];
uint16_t disp[PLOT_W];          // displayed samples in mV
int16_t  ys[PLOT_W];
float freqHz = -1;
int vMaxMv = 0, vMinMv = 0;

// ---------- Buttons (own task: 0 = OK, 1 = PLUS, 2 = MINUS, 3 = BACK) ----------
const uint8_t BTN_PINS[4] = {BTN_OK, BTN_PLUS, BTN_MINUS, BTN_BACK};
volatile int8_t btnEv[4] = {0, 0, 0, 0};
volatile bool   btnIgnored[4] = {false, false, false, false};
bool btnWarn = false;

bool isPressed(int pin) { return digitalRead(pin) == (BTN_ACTIVE_HIGH ? HIGH : LOW); }

void setupBtnPin(int pin) {
#if BTN_ACTIVE_HIGH
  if (pin >= 34) pinMode(pin, INPUT);            // input-only pins: no internal resistor
  else           pinMode(pin, INPUT_PULLDOWN);
#else
  pinMode(pin, INPUT_PULLUP);
#endif
}

void pushEv(int i) { if (btnEv[i] < 3) btnEv[i] = btnEv[i] + 1; }
int  takeEv(int i) { int n = btnEv[i]; btnEv[i] = 0; return n; }

void buttonTask(void *) {
  bool state[4], raw[4];
  uint32_t tRaw[4], nextRep[4];
  uint32_t now = millis();
  for (int i = 0; i < 4; i++) {
    state[i] = raw[i] = isPressed(BTN_PINS[i]);  // a button already held at power-up gives no event
    tRaw[i] = now;
    nextRep[i] = 0;
  }
  for (;;) {
    now = millis();
    for (int i = 0; i < 4; i++) {
      if (btnIgnored[i]) continue;
      bool r = isPressed(BTN_PINS[i]);
      if (r != raw[i]) { raw[i] = r; tRaw[i] = now; }
      else if (r != state[i] && (now - tRaw[i]) >= DEBOUNCE_MS) {
        state[i] = r;
        if (r) { pushEv(i); nextRep[i] = now + REPEAT_DELAY_MS; }
      }
      if (state[i] && (i == 1 || i == 2) && now >= nextRep[i]) {
        pushEv(i);
        nextRep[i] = now + REPEAT_MS;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(3));
  }
}

void checkFloatingButtons() {
  for (int i = 0; i < 4; i++) {
    int flips = 0;
    bool last = isPressed(BTN_PINS[i]);
    for (int k = 0; k < 400; k++) {
      bool v = isPressed(BTN_PINS[i]);
      if (v != last) { flips++; last = v; }
      delayMicroseconds(500);
    }
    if (flips > 8) {
      btnIgnored[i] = true;
      btnWarn = true;
    }
  }
}

// ---------- Strip drawing helpers (screen coordinates -> strip) ----------
inline void gLine(int x0, int y0, int x1, int y1, uint16_t c) { cv.drawLine(x0, y0 - oy, x1, y1 - oy, c); }
inline void gPix(int x, int y, uint16_t c)                    { cv.drawPixel(x, y - oy, c); }
inline void gText(int x, int y, const char* s, uint16_t fg, uint16_t bg) {
  cv.setTextColor(fg, bg);
  cv.setCursor(x, y - oy);
  cv.print(s);
}
void gFill(int x, int y, int w, int h, uint16_t c) {
  int y0 = max(y, oy), y1 = min(y + h, oy + STRIP_H);
  if (y1 <= y0) return;
  cv.fillRect(x, y0 - oy, w, y1 - y0, c);
}

// ---------- I2S / ADC ----------
void startI2S(uint32_t rate) {
  if (curRate) {
    i2s_adc_disable(I2S_NUM_0);
    i2s_driver_uninstall(I2S_NUM_0);
  }
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
  cfg.sample_rate = rate;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CH);
  adc1_config_channel_atten(ADC_CH, ADC_ATTEN_DB_11);
  i2s_adc_enable(I2S_NUM_0);
  curRate = rate;
}

void applyTimebase() {
  if (TB[tb].rate != curRate) startI2S(TB[tb].rate);
}

// Throw away the old samples that piled up in the DMA buffers while the screen was drawing
void flushDma() {
  static uint16_t dummy[256];
  size_t br = 0;
  for (int i = 0; i < 4; i++) i2s_read(I2S_NUM_0, dummy, sizeof(dummy), &br, pdMS_TO_TICKS(200));
}

// ---------- Capture & analysis ----------
float measureFreq(const uint16_t *s, int n, float sampleUs) {
  int mx = 0, mi = 100000;
  for (int i = 0; i < n; i++) { if (s[i] > mx) mx = s[i]; if (s[i] < mi) mi = s[i]; }
  int pp = mx - mi;
  if (pp < 100) return -1;                              // less than 100 mV peak to peak
  float mid = (mx + mi) / 2.0f, hy = pp / 6.0f;
  bool low = false;
  int edges = 0;
  float first = 0, last = 0;
  for (int i = 1; i < n; i++) {
    float a = s[i - 1], b = s[i];
    if (b < mid - hy) low = true;
    else if (low && b >= mid && a < mid) {
      float pos = (i - 1) + (mid - a) / (b - a);
      if (edges == 0) first = pos;
      last = pos;
      edges++;
      low = false;
    }
  }
  if (edges < 2) return -1;
  float per = (last - first) / (edges - 1);
  return 1e6f / (per * sampleUs);
}

// returns: 1 = triggered, 2 = AUTO free-run, 0 = waiting (old picture kept), -1 = read error
int capture() {
  const int decim = TB[tb].decim;
  const float sampleUs = 1e6f * decim / TB[tb].rate;

  flushDma();
  size_t need = (size_t)TRACE_N * decim;
  size_t br = 0;
  i2s_read(I2S_NUM_0, rawBuf, need * sizeof(uint16_t), &br, pdMS_TO_TICKS(4000));
  int n = (int)(br / 2) / decim;
  if (n > TRACE_N) n = TRACE_N;
  if (n < PLOT_W + PRE_N) return -1;

  for (int j = 0; j < n; j++) {
    uint32_t sum = 0;
    for (int k = 0; k < decim; k++) sum += rawBuf[j * decim + k] & 0x0FFF;   // average = cleaner slow timebases
    tmpMv[j] = esp_adc_cal_raw_to_voltage(sum / decim, &adcChars);
  }

  freqHz = measureFreq(tmpMv, n, sampleUs);

  // trigger search: crossing with hysteresis, at least PRE_N samples from the start
  const int trig = trgU * 50;                           // mV
  const int hi = n - (PLOT_W - PRE_N) - 1;
  int c = -1;
  bool armed = false;
  for (int j = 0; j <= hi; j++) {
    int v = tmpMv[j];
    if (trgRising) {
      if (v < trig - HYST_MV) armed = true;
      else if (armed && v >= trig) { if (j >= PRE_N) { c = j; break; } armed = false; }
    } else {
      if (v > trig + HYST_MV) armed = true;
      else if (armed && v <= trig) { if (j >= PRE_N) { c = j; break; } armed = false; }
    }
  }

  int start;
  int result;
  if (c >= 0)            { start = c - PRE_N; result = 1; }
  else if (trgMode == 0) { start = 0;         result = 2; }
  else return 0;

  for (int i = 0; i < PLOT_W; i++) disp[i] = tmpMv[start + i];
  dataValid = true;

  vMaxMv = 0; vMinMv = 100000;
  for (int i = 0; i < PLOT_W; i++) {
    if (disp[i] > vMaxMv) vMaxMv = disp[i];
    if (disp[i] < vMinMv) vMinMv = disp[i];
  }
  return result;
}

// ---------- Drawing ----------
float offV() { return offT * 0.1f; }
float divUs() { return (1e6f * TB[tb].decim / TB[tb].rate) * SAMPLES_PER_DIV; }

int yOfMv(int mv) {
  int y = PLOT_Y + PLOT_H - (int)((mv / 1000.0f - offV()) / VDIV[vi] * PX_DIV_Y);
  return constrain(y, PLOT_Y, PLOT_Y + PLOT_H);
}

void computeYs() {
  for (int i = 0; i < PLOT_W; i++) ys[i] = (int16_t)yOfMv(disp[i]);
}

void drawGrid() {
  // horizontal grid lines (dotted); the centre line is brighter and denser
  for (int j = 1; j < 8; j++) {
    int y = PLOT_Y + j * PX_DIV_Y;
    if (y < oy || y >= oy + STRIP_H) continue;
    bool mid = (j == 4);
    for (int x = PLOT_X; x <= PLOT_X + PLOT_W; x += (mid ? 2 : 4)) gPix(x, y, mid ? C_GRID2 : C_GRID);
  }
  int yTop = max(PLOT_Y, oy);
  int yBot = min(PLOT_Y + PLOT_H, oy + STRIP_H - 1);
  if (yTop <= yBot) {
    // vertical grid lines (dotted)
    for (int i = 1; i < 10; i++) {
      int x = PLOT_X + i * PX_DIV_X;
      bool mid = (i == 5);
      int st = mid ? 2 : 4;
      for (int y = yTop; y <= yBot; y++) if (((y - PLOT_Y) % st) == 0) gPix(x, y, mid ? C_GRID2 : C_GRID);
    }
    // tick marks on the vertical centre axis (5 per division)
    int xc = PLOT_X + 5 * PX_DIV_X;
    for (int y = yTop; y <= yBot; y++)
      if (((y - PLOT_Y) % (PX_DIV_Y / 5)) == 0)
        for (int d = -2; d <= 2; d++) gPix(xc + d, y, C_FRAME);
    // frame sides
    gLine(PLOT_X, yTop, PLOT_X, yBot, C_FRAME);
    gLine(PLOT_X + PLOT_W, yTop, PLOT_X + PLOT_W, yBot, C_FRAME);
  }
  // tick marks on the horizontal centre axis (5 per division)
  int yc = PLOT_Y + 4 * PX_DIV_Y;
  if (yc + 2 >= oy && yc - 2 < oy + STRIP_H)
    for (int x = PLOT_X; x <= PLOT_X + PLOT_W; x += PX_DIV_X / 5)
      for (int d = -2; d <= 2; d++) gPix(x, yc + d, C_FRAME);
  // frame top and bottom
  if (PLOT_Y >= oy && PLOT_Y < oy + STRIP_H) cv.drawFastHLine(PLOT_X, PLOT_Y - oy, PLOT_W + 1, C_FRAME);
  int yb = PLOT_Y + PLOT_H;
  if (yb >= oy && yb < oy + STRIP_H) cv.drawFastHLine(PLOT_X, yb - oy, PLOT_W + 1, C_FRAME);
}

void drawTrace() {
  if (!dataValid) return;
  int top = oy, bot = oy + STRIP_H - 1;
  for (int i = 1; i < PLOT_W; i++) {
    int y0 = ys[i - 1], y1 = ys[i];
    if ((y0 < top - 1 && y1 < top - 1) || (y0 > bot && y1 > bot)) continue;
    gLine(PLOT_X + i - 1, y0,     PLOT_X + i, y1,     C_TRACE);
    gLine(PLOT_X + i - 1, y0 + 1, PLOT_X + i, y1 + 1, C_TRACE);
  }
}

void drawMarkers() {
  int yT = yOfMv(trgU * 50);                          // trigger level arrow (left)
  cv.fillTriangle(2, yT - 4 - oy, 2, yT + 4 - oy, 9, yT - oy, C_CYAN);
  int xT = PLOT_X + PRE_N;                            // trigger position arrow (top)
  cv.fillTriangle(xT - 3, PLOT_Y + 1 - oy, xT + 3, PLOT_Y + 1 - oy, xT, PLOT_Y + 7 - oy, C_ORANGE);
}

uint16_t curColor(int idx) { return (curSel == idx) ? C_WHITE : C_CYAN; }

void drawCursorLines() {
  if (curV1 >= oy && curV1 < oy + STRIP_H) cv.drawFastHLine(PLOT_X, curV1 - oy, PLOT_W + 1, curColor(1));
  if (curV2 >= oy && curV2 < oy + STRIP_H) cv.drawFastHLine(PLOT_X, curV2 - oy, PLOT_W + 1, curColor(2));
  cv.drawLine(curT1, PLOT_Y - oy, curT1, PLOT_Y + PLOT_H - oy, curColor(3));
  cv.drawLine(curT2, PLOT_Y - oy, curT2, PLOT_Y + PLOT_H - oy, curColor(4));
}

void fmtFreq(char *out, size_t n, float hz) {
  if (hz < 0)             snprintf(out, n, "---");
  else if (hz >= 1000.0f) snprintf(out, n, "%.2fkHz", hz / 1000.0f);
  else                    snprintf(out, n, "%.1fHz", hz);
}

void fmtTime(char *out, size_t n, float us) {
  if (us >= 1000.0f) snprintf(out, n, "%.2fms", us / 1000.0f);
  else               snprintf(out, n, "%.1fus", us);
}

void drawItem(int x, int y, int idx, const char* text) {
  bool s = running && sel == idx;
  gText(x, y, text, s ? C_BLACK : C_WHITE, s ? C_YELLOW : C_HDR);
}

void drawBars() {
  char b[48], f[16], t[16];
  cv.setTextSize(1);

  gFill(0, 0, SCR_W, HDR_H, C_HDR);                     // header bar
  gFill(0, BAR_Y, SCR_W, SCR_H - BAR_Y, C_HDR);         // bottom bar

  snprintf(b, sizeof b, "T:%s", TB[tb].label);                 drawItem(4,   3, M_TIME, b);
  snprintf(b, sizeof b, "V:%s", VLBL[vi]);                     drawItem(66,  3, M_VDIV, b);
  snprintf(b, sizeof b, "O:%.1fV", offV());                    drawItem(116, 3, M_OFFS, b);
  snprintf(b, sizeof b, "L:%.2fV", trgU * 0.05f);              drawItem(170, 3, M_TRIG, b);
  snprintf(b, sizeof b, "E:%s", trgRising ? "RISE" : "FALL");  drawItem(4,   12, M_EDGE, b);
  snprintf(b, sizeof b, "M:%s", trgMode == 0 ? "AUTO" : (trgMode == 1 ? "NORM" : "SNGL")); drawItem(66, 12, M_MODE, b);

  gText(272, 3, running ? "RUN" : "STOP", running ? C_GREEN : C_RED, C_HDR);
  if (btnWarn) gText(272, 12, "BTN!", C_RED, C_HDR);

  if (running) {
    if (triggered)         gText(116, 12, "TRG'D", C_CYAN,   C_HDR);
    else if (trgMode == 0) gText(116, 12, "AUTO",  C_ORANGE, C_HDR);
    else                   gText(116, 12, "WAIT",  C_ORANGE, C_HDR);
  } else {
    gText(116, 12, "HOLD", C_RED, C_HDR);
  }

  if (running) {
    snprintf(b, sizeof b, "Max %.2f", vMaxMv / 1000.0f);               gText(4,   228, b, C_WHITE, C_HDR);
    snprintf(b, sizeof b, "Min %.2f", vMinMv / 1000.0f);               gText(70,  228, b, C_WHITE, C_HDR);
    snprintf(b, sizeof b, "Vpp %.2f", (vMaxMv - vMinMv) / 1000.0f);    gText(136, 228, b, C_WHITE, C_HDR);
    fmtFreq(f, sizeof f, freqHz);
    snprintf(b, sizeof b, "F:%s", f);                                  gText(205, 228, b, C_YELLOW, C_HDR);
  } else {
    float dV  = fabsf((float)(curV1 - curV2)) / PX_DIV_Y * VDIV[vi];
    float dTu = fabsf((float)(curT1 - curT2)) / PX_DIV_X * divUs();
    fmtTime(t, sizeof t, dTu);
    fmtFreq(f, sizeof f, dTu > 0 ? 1e6f / dTu : -1.0f);
    snprintf(b, sizeof b, "dV=%.2fV dT=%s f=%s", dV, t, f);
    gText(4, 228, b, C_WHITE, C_HDR);
  }
}

void render() {
  if (dataValid) computeYs();
  for (oy = 0; oy < SCR_H; oy += STRIP_H) {
    cv.fillScreen(C_BLACK);
    drawGrid();
    drawTrace();
    drawMarkers();
    if (!running) drawCursorLines();
    drawBars();
    tft.startWrite();
    tft.setAddrWindow(0, oy, SCR_W, STRIP_H);
    tft.writePixels(cv.getBuffer(), (uint32_t)SCR_W * STRIP_H);
    tft.endWrite();
  }
}

// ---------- Button actions ----------
void adjust(int d) {
  switch (sel) {
    case M_TIME: {
      int nt = constrain(tb + d, 0, TB_COUNT - 1);
      if (nt != tb) { tb = nt; applyTimebase(); dataValid = false; }
      break;
    }
    case M_VDIV: vi   = constrain(vi + d, 0, 3);      break;
    case M_OFFS: offT = constrain(offT + d, 0, 30);   break;
    case M_TRIG: trgU = constrain(trgU + d, 1, 65);   break;
    case M_EDGE: trgRising = !trgRising;              break;
    case M_MODE: trgMode = ((trgMode + d) % 3 + 3) % 3; break;
  }
}

void moveCursor(int d) {
  switch (curSel) {
    case 1: curV1 = constrain(curV1 + d, PLOT_Y, PLOT_Y + PLOT_H); break;
    case 2: curV2 = constrain(curV2 + d, PLOT_Y, PLOT_Y + PLOT_H); break;
    case 3: curT1 = constrain(curT1 + d, PLOT_X, PLOT_X + PLOT_W); break;
    case 4: curT2 = constrain(curT2 + d, PLOT_X, PLOT_X + PLOT_W); break;
  }
}

void processButtons() {
  int nSet = takeEv(0), nPlus = takeEv(1), nMinus = takeEv(2), nRun = takeEv(3);

  if (nRun & 1) {                                       // RUN / STOP
    running = !running;
    curSel = 0;
    if (running) triggered = false;
  }
  if (nSet) {
    if (running) sel = (sel + nSet) % M_COUNT;
    else         curSel = (curSel + nSet) % 5;
  }
  int d = nPlus - nMinus;
  if (d != 0) {
    if (running) adjust(d);
    else         moveCursor(d * 3);
  }
}

// ---------- Setup / loop ----------
void setup() {
  for (int i = 0; i < 4; i++) setupBtnPin(BTN_PINS[i]);
  delay(50);
  checkFloatingButtons();                               // a floating pin is ignored, not allowed to bounce
  xTaskCreatePinnedToCore(buttonTask, "buttons", 3072, NULL, 1, NULL, 0);

  // Manual reset pulse (same as the working test sketch)
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH); delay(50);
  digitalWrite(TFT_RST, LOW);  delay(100);
  digitalWrite(TFT_RST, HIGH); delay(150);

#if USE_SPI_MODE3
  tft.init(240, 320, SPI_MODE3);
#else
  tft.init(240, 320);
#endif
#if INVERT_MODE == 1
  tft.invertDisplay(true);
#elif INVERT_MODE == 2
  tft.invertDisplay(false);
#endif
  tft.setRotation(ROTATION);
  tft.fillScreen(C_BLACK);

  if (cv.getBuffer() == NULL) {
    tft.setTextColor(C_RED);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.print("Canvas alloc failed");
    while (true) delay(1000);
  }

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adcChars);

#if ENABLE_TEST_SIGNAL
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(TEST_PIN, 1000, 8);
    ledcWrite(TEST_PIN, 128);
  #else
    ledcSetup(0, 1000, 8);                              // 1 kHz, 50 % duty on GPIO25
    ledcAttachPin(TEST_PIN, 0);
    ledcWrite(0, 128);
  #endif
#endif

  startI2S(TB[tb].rate);
}

void loop() {
  processButtons();

  if (running) {
    int r = capture();
    if (r == 1) {
      triggered = true;
      if (trgMode == 2) running = false;                // SINGLE: stop after the capture
    } else if (r == 2 || r == 0) {
      triggered = false;
    }
  }

  render();
}