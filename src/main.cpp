#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#define LEDPIN 25

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 21
#define OLED_SCL 22

#define TMP_SENS 34

#define WET_TRESH 1300
#define DRY_TRESH 3100
#define INTERVALS (DRY_TRESH - WET_TRESH) / 3

unsigned int counter = 0;
char TTN_response[30];

SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

#define BLINK_GPIO CONFIG_BLINK_GPIO

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes.

// Copy the value from Device EUI from the TTN console in LSB mode.
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 120;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32} // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void do_send(osjob_t *j)
{
  int rawHumidity = adc1_get_raw(ADC1_CHANNEL_6);
  String humidity = "";

  if (rawHumidity > WET_TRESH && rawHumidity < (WET_TRESH + INTERVALS))
  {
    humidity = "Very wet";
  }
  else if (rawHumidity > (WET_TRESH + INTERVALS) && rawHumidity < (DRY_TRESH - INTERVALS))
  {
    humidity = "Wet";
  }
  else if (rawHumidity < DRY_TRESH && rawHumidity > (DRY_TRESH - INTERVALS))
  {
    humidity = "Dry";
  }

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (uint8_t *)(&humidity), sizeof(humidity) - 1, 0);
    Serial.println(F("Sending uplink packet..."));
    digitalWrite(LEDPIN, HIGH);
    display.clear();
    display.drawString(0, 0, "Sending uplink packet...");
    display.drawString(0, 22, TTN_response);
    display.drawString(0, 50, String(++counter));
    display.display();
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev)
{
  Serial.print("os-time: ");
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.print(ev);
  switch (ev)
  {
  case EV_TXCOMPLETE:
  {
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    display.clear();
    display.drawString(0, 0, "EV_TXCOMPLETE event!");

    display.clear();
    int num = adc1_get_raw(ADC1_CHANNEL_6);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Initialized, value:");
    Serial.println("ADC initialized");
    Serial.println(num);
    display.setFont(ArialMT_Plain_24);
    char *buffer = (char *)malloc(80);
    display.drawStringf(0, 12, buffer, "%d", num);
    display.display();
    display.setFont(ArialMT_Plain_10);

    if (LMIC.txrxFlags & TXRX_ACK)
    {
      Serial.println(F("Received ack"));
      display.drawString(0, 20, "Received ACK.");
    }

    // Schedule next transmission
    ostime_t nextTic = os_getTime() + sec2osticks(TX_INTERVAL);
    Serial.println(F("TX_INTERVAL: "));
    Serial.println(TX_INTERVAL);
    Serial.println(F("\nsec2osticks(TX_INTERVAL)"));
    Serial.println(sec2osticks(TX_INTERVAL));
    Serial.println(F("os-time: "));
    Serial.println(os_getTime());
    Serial.println(F("next tic: "));
    Serial.println(nextTic);

    os_setTimedCallback(&sendjob, nextTic, do_send);
    digitalWrite(LEDPIN, LOW);
    display.drawString(0, 50, "counter " + String(counter));
    display.display();
    break;
  }
  case EV_JOINING:
    Serial.println(F("EV_JOINING: -> Joining..."));
    display.drawString(0, 16, "OTAA joining....");
    display.display();
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    display.clear();
    display.drawString(0, 0, "Joined!");
    display.display();
    // Disable link check validation (automatically enabled
    // during join, but not supported by TTN at this time).
    LMIC_setLinkCheckMode(0);

    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  default:
    Serial.println(F("Unknown event"));
    break;
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2500); // Give time to the serial monitor to pick up
  Serial.println(F("Starting..."));

  // Use the Blue pin to signal transmission.
  pinMode(LEDPIN, OUTPUT);

  //Set up and reset the OLED
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(50);
  digitalWrite(OLED_RESET, HIGH);

  //Set up temp sensor
  if (adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_6) == ESP_OK)
  {
  pinMode(TMP_SENS, INPUT);
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  }

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.drawString(0, 0, "Starting....");
  display.display();
  digitalWrite(LEDPIN, HIGH);

  TTN_response[0] = '-';
  TTN_response[1] = 0;

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF9, 14);

  // Start job
  do_send(&sendjob); // Will fire up also the join
}

void loop()
{
  os_runloop_once();
}