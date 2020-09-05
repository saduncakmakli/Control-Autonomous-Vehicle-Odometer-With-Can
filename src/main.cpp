#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

const short teker = 185;
const short tur_pulse = 768;

const short EncoderA = 2;
const short EncoderB = 3;

const short spiCSPin = 10; //12 MISO, 11 MOSI, 13 SCK
MCP_CAN CAN(spiCSPin);

volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
volatile double hiz;
volatile double yol;

void ai0()
{
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW)
  {
    counter++;
  }
  else
  {
    counter--;
  }
}

void ai1()
{
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW)
  {
    counter--;
  }
  else
  {
    counter++;
  }
}

void setup()
{
  Serial.begin(9600);

  //CAN CONNETION BEGIN
  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS Init Failed");
    delay(100);
  }
  Serial.println("CAN BUS Init OK!");

  pinMode(EncoderA, INPUT_PULLUP); // internal pullup input pin EncoderA
  pinMode(EncoderB, INPUT_PULLUP); // internal pullup input pin EncoderB

  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(EncoderA), ai0, RISING);

  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(EncoderB), ai1, RISING);

  cli();
  /* Ayarlamaların yapılabilmesi için öncelikle kesmeler durduruldu */

  /* Timer1 kesmesi saniyede bir çalışacak şekilde ayarlanacaktır (1 Hz)*/
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 15624;
  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
  TCCR1B |= (1 << WGM12);
  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  /* Timer1 kesmesi aktif hale getirildi */

  sei();
  /* Timer1 kesmesinin çalışabilmesi için tüm kesmeler aktif hale getirildi */
}

void loop()
{
}

ISR(TIMER1_COMPA_vect)
{
  if (counter != temp)
  {
    int fark = counter - temp;
    double oran = (double)fark / tur_pulse;
    hiz = oran * teker * 0.036;
    if (hiz < 0) hiz = hiz * -1;
    yol = (double)counter * (double)teker / (double)tur_pulse; //cm
    
    Serial.println(hiz);
    Serial.println(yol);
    temp = counter;
  }

  byte data_byte_0 = map(hiz, 0, 60, 0, 255); //240 adresinden hız bilgisi
  byte stmp[1] = {data_byte_0};
  CAN.sendMsgBuf(0xF0, 0, 1, stmp); //240 adresinden hız bilgisi yazdırılıyor.
}
