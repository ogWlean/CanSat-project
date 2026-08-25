#include <SoftwareSerial.h>

// ============================================================
// E32 UART
// ============================================================
//
// E32 TXD -> Arduino UNO D2 (RX)
// E32 RXD -> Arduino UNO D3 (TX)
//
// ============================================================

#define RX_PIN 2
#define TX_PIN 3

SoftwareSerial e32(RX_PIN, TX_PIN);


// ============================================================
// APPLICATION FRAME
// ============================================================
//
// This must exactly match the ESP32 transmitter.
//
// ============================================================

#define SYNC0 0xAA
#define SYNC1 0x55


struct __attribute__((packed)) DataPacket
{
  uint16_t packetID;

  float a_total;

  float temperature;
  float humidity;

  float pressure;
  float altitude;

  uint8_t checksum;
};


DataPacket data;


// ============================================================
// RECEIVER STATE MACHINE
// ============================================================

enum RecvState
{
  WAIT_SYNC0,
  WAIT_SYNC1,
  READ_PACKET
};

RecvState state = WAIT_SYNC0;

uint8_t buf[sizeof(DataPacket)];

uint8_t bufIndex = 0;

unsigned long packetStart = 0;

unsigned long lastReceived = 0;

bool connected = false;

int32_t lastID = -1;


// ============================================================
// CRC-8
// ============================================================
//
// Same polynomial as ESP32:
//
// x^8 + x^2 + x + 1
//
// Polynomial = 0x07
//
// ============================================================

uint8_t crc8(
  const uint8_t* b,
  size_t len
)
{
  uint8_t crc = 0x00;

  for (size_t i = 0; i < len; i++)
  {
    crc ^= b[i];

    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x80)
      {
        crc =
          (uint8_t)((crc << 1) ^ 0x07);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}


// ============================================================
// SEND VERIFIED FRAME TO PC / PYTHON
// ============================================================
//
// USB Serial:
//
// AA 55 + DataPacket
//
// This preserves your existing Python bridge format.
//
// ============================================================

void sendToPython()
{
  Serial.write(
    SYNC0
  );

  Serial.write(
    SYNC1
  );

  Serial.write(
    (uint8_t*)&data,
    sizeof(DataPacket)
  );
}


// ============================================================
// PRINT TELEMETRY
// ============================================================
//
// IMPORTANT:
//
// Your Python bridge currently expects raw binary on Serial.
// Therefore don't print normal text to Serial.
//
// If you want debugging, use another serial interface.
// ============================================================

void printPacketDebug()
{
  // Intentionally disabled.
  //
  // Serial is being used as the binary Python bridge.
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  // USB -> Python
  Serial.begin(115200);

  // UNO <-> E32
  e32.begin(9600);

  state = WAIT_SYNC0;

  bufIndex = 0;
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  unsigned long now =
    millis();


  // ==========================================================
  // CONNECTION TIMEOUT
  // ==========================================================

  if (connected &&
      now - lastReceived > 8000)
  {
    connected = false;

    state = WAIT_SYNC0;

    bufIndex = 0;
  }


  // ==========================================================
  // PACKET TIMEOUT
  // ==========================================================

  if (state != WAIT_SYNC0 &&
      now - packetStart > 3000)
  {
    state = WAIT_SYNC0;

    bufIndex = 0;
  }


  // ==========================================================
  // READ E32
  // ==========================================================

  while (e32.available())
  {
    uint8_t b =
      e32.read();


    switch (state)
    {
      // ------------------------------------------------------
      // WAIT FOR 0xAA
      // ------------------------------------------------------

      case WAIT_SYNC0:

        if (b == SYNC0)
        {
          state = WAIT_SYNC1;

          packetStart =
            millis();
        }

        break;


      // ------------------------------------------------------
      // WAIT FOR 0x55
      // ------------------------------------------------------

      case WAIT_SYNC1:

        if (b == SYNC1)
        {
          state = READ_PACKET;

          bufIndex = 0;
        }

        else if (b == SYNC0)
        {
          // Another AA.
          // Keep waiting for 55.

          packetStart =
            millis();
        }

        else
        {
          state =
            WAIT_SYNC0;
        }

        break;


      // ------------------------------------------------------
      // READ COMPLETE DATA PACKET
      // ------------------------------------------------------

      case READ_PACKET:

        buf[bufIndex++] =
          b;


        if (bufIndex >=
            sizeof(DataPacket))
        {
          // Copy received bytes into structure.

          memcpy(
            &data,
            buf,
            sizeof(DataPacket)
          );


          // --------------------------------------------------
          // CRC CHECK
          // --------------------------------------------------

          uint8_t expected =
            crc8(
              buf,
              sizeof(DataPacket) - 1
            );


          if (data.checksum ==
              expected)
          {
            // Valid packet.

            lastReceived =
              millis();


            connected =
              true;


            // ------------------------------------------------
            // DEDUPLICATION
            // ------------------------------------------------

            if ((int32_t)data.packetID !=
                lastID)
            {
              lastID =
                data.packetID;


              // Send ONLY verified packet to Python.

              sendToPython();
            }
          }


          // Reset parser.

          state =
            WAIT_SYNC0;

          bufIndex =
            0;
        }

        break;
    }
  }
}