/*
 * tft_status -- drone status readout for a parallel TFT LCD shield on an
 * Arduino Uno (the common 2.4"/2.8" MCUFRIEND-style shield: ILI9341,
 * ILI9325, HX8347, SPFD5408 and friends, auto-detected at runtime).
 *
 * The shield occupies D2-D9 and A0-A4. D0/D1 stay free, so the USB serial
 * link to the Jetson still works -- that is what feeds this sketch.
 *
 * REQUIRED LIBRARIES (Arduino IDE -> Library Manager):
 *   - MCUFRIEND_kbv  by David Prentice
 *   - Adafruit GFX Library  by Adafruit
 *
 * Protocol (newline terminated, 115200 baud). The Arduino does no
 * interpretation -- all formatting happens in the ROS 2 lcd_status node, so
 * the layout can change without reflashing the board:
 *
 *   T:<text>     banner text (the big stage name across the top)
 *   S:<0-3>      banner colour: 0 idle/grey, 1 ok/green, 2 busy/amber,
 *                3 alarm/red. Send before T: for the colour to apply.
 *   1:<text> ..  6:<text>   body rows, top to bottom
 *   C  (or C:)   clear the whole screen
 *
 * Rows are redrawn only when their text changes, because a full repaint on
 * an 8-bit parallel bus is slow enough to be visibly laggy.
 *
 * If nothing valid arrives for LINK_TIMEOUT_MS the screen goes red with
 * "NO LINK", so a dead Jetson or an unplugged cable is obvious rather than
 * leaving a frozen stale status on screen.
 */

#include <MCUFRIEND_kbv.h>
#include <Adafruit_GFX.h>

MCUFRIEND_kbv tft;

// ---- layout ---------------------------------------------------------------
const int16_t SCREEN_W = 240;
const int16_t SCREEN_H = 320;

const int16_t BANNER_H = 46;
const uint8_t BANNER_TEXT_SIZE = 3;   // 18x24 px per char
const uint8_t BODY_TEXT_SIZE = 2;     // 12x16 px per char -> 20 cols
const int16_t BODY_TOP = BANNER_H + 10;
const int16_t BODY_ROW_H = 24;
const int16_t BODY_LEFT = 6;

const uint8_t NUM_ROWS = 6;
const uint8_t MAX_TEXT = 24;          // longest string we will store per row

// ---- colours --------------------------------------------------------------
const uint16_t C_BLACK = 0x0000;
const uint16_t C_WHITE = 0xFFFF;
const uint16_t C_GREY = 0x8410;
const uint16_t C_GREEN = 0x07E0;
const uint16_t C_AMBER = 0xFD20;
const uint16_t C_RED = 0xF800;
const uint16_t C_BG = 0x0000;

const unsigned long LINK_TIMEOUT_MS = 3000;

// ---- state ----------------------------------------------------------------
char rowText[NUM_ROWS][MAX_TEXT + 1];
char bannerText[MAX_TEXT + 1];
uint8_t bannerSeverity = 0;

char buffer[48];
uint8_t bufferLen = 0;

unsigned long lastMessageMs = 0;
bool linkLost = false;

uint16_t severityColour(uint8_t s) {
  switch (s) {
    case 1: return C_GREEN;
    case 2: return C_AMBER;
    case 3: return C_RED;
    default: return C_GREY;
  }
}

void drawBanner() {
  uint16_t colour = severityColour(bannerSeverity);
  tft.fillRect(0, 0, SCREEN_W, BANNER_H, colour);
  // Dark text on the light amber/green fills, white on grey and red.
  tft.setTextColor((bannerSeverity == 1 || bannerSeverity == 2) ? C_BLACK : C_WHITE);
  tft.setTextSize(BANNER_TEXT_SIZE);
  tft.setCursor(6, (BANNER_H - 8 * BANNER_TEXT_SIZE) / 2);
  tft.print(bannerText);
}

void drawRow(uint8_t row) {
  if (row >= NUM_ROWS) return;
  int16_t y = BODY_TOP + row * BODY_ROW_H;
  // Blank the row first; without this a shorter string leaves the tail of
  // the previous one on screen.
  tft.fillRect(0, y, SCREEN_W, BODY_ROW_H, C_BG);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(BODY_TEXT_SIZE);
  tft.setCursor(BODY_LEFT, y);
  tft.print(rowText[row]);
}

void clearAll() {
  tft.fillScreen(C_BG);
  for (uint8_t i = 0; i < NUM_ROWS; i++) rowText[i][0] = '\0';
  bannerText[0] = '\0';
  bannerSeverity = 0;
  drawBanner();
}

void copyTruncated(char *dest, const char *src) {
  uint8_t i = 0;
  while (i < MAX_TEXT && src[i] != '\0') {
    dest[i] = src[i];
    i++;
  }
  dest[i] = '\0';
}

void handleLine(char *line) {
  lastMessageMs = millis();

  if (linkLost) {
    // Coming back from the NO LINK screen: repaint everything we know.
    linkLost = false;
    tft.fillScreen(C_BG);
    drawBanner();
    for (uint8_t i = 0; i < NUM_ROWS; i++) drawRow(i);
  }

  // Accept both "C" and "C:" so the sender can use the same key:value form
  // it uses for everything else.
  if (line[0] == 'C' && (line[1] == '\0' || (line[1] == ':' && line[2] == '\0'))) {
    clearAll();
    return;
  }

  if (line[0] == 'S' && line[1] == ':') {
    uint8_t s = line[2] - '0';
    if (s <= 3 && s != bannerSeverity) {
      bannerSeverity = s;
      drawBanner();
    }
    return;
  }

  if (line[0] == 'T' && line[1] == ':') {
    if (strncmp(bannerText, line + 2, MAX_TEXT) != 0) {
      copyTruncated(bannerText, line + 2);
      drawBanner();
    }
    return;
  }

  if (line[0] >= '1' && line[0] <= '0' + NUM_ROWS && line[1] == ':') {
    uint8_t row = line[0] - '1';
    if (strncmp(rowText[row], line + 2, MAX_TEXT) != 0) {
      copyTruncated(rowText[row], line + 2);
      drawRow(row);
    }
    return;
  }
}

void showNoLink() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, SCREEN_W, BANNER_H, C_RED);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(BANNER_TEXT_SIZE);
  tft.setCursor(6, (BANNER_H - 8 * BANNER_TEXT_SIZE) / 2);
  tft.print("NO LINK");
  tft.setTextSize(BODY_TEXT_SIZE);
  tft.setCursor(BODY_LEFT, BODY_TOP);
  tft.print("jetson silent");
  tft.setCursor(BODY_LEFT, BODY_TOP + BODY_ROW_H);
  tft.print("check usb / node");
}

void setup() {
  Serial.begin(115200);

  uint16_t id = tft.readID();
  // Some shields report 0x0000 or 0xD3D3; 0x9341 is the usual fallback that
  // drives them correctly anyway.
  if (id == 0x0000 || id == 0xD3D3 || id == 0xFFFF) id = 0x9341;
  tft.begin(id);
  tft.setRotation(0);            // portrait, 240x320
  tft.fillScreen(C_BG);

  for (uint8_t i = 0; i < NUM_ROWS; i++) rowText[i][0] = '\0';

  copyTruncated(bannerText, "BOOT");
  bannerSeverity = 0;
  drawBanner();

  copyTruncated(rowText[0], "drone_testing");
  copyTruncated(rowText[1], "waiting for ROS");
  drawRow(0);
  drawRow(1);

  lastMessageMs = millis();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      buffer[bufferLen] = '\0';
      if (bufferLen > 0) handleLine(buffer);
      bufferLen = 0;
      continue;
    }

    // Overlong lines are dropped rather than wrapped, so a garbled byte
    // stream cannot desync the parser permanently.
    if (bufferLen < sizeof(buffer) - 1) {
      buffer[bufferLen++] = c;
    } else {
      bufferLen = 0;
    }
  }

  if (!linkLost && (millis() - lastMessageMs) > LINK_TIMEOUT_MS) {
    linkLost = true;
    showNoLink();
  }
}
