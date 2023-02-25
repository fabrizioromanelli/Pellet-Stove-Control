void getDBG(byte input)
{
  const byte readByte = 0x20;
  StoveSerial.write(readByte);
  delay(1);
  StoveSerial.write(input);
  digitalWrite(ENABLE_RX, LOW);
  delay(80);
  uint8_t rxCount = 0;
  stoveRxData[0] = 0x00;
  stoveRxData[1] = 0x00;
  while (StoveSerial.available())
  {
    stoveRxData[rxCount] = StoveSerial.read();
    rxCount++;
  }
  digitalWrite(ENABLE_RX, HIGH);
  if (rxCount == 2)
  {
    byte val = stoveRxData[1];
    byte checksum = stoveRxData[0];
    byte param = checksum - val;
    Serial.printf("Param=%01x value=%01x ", param, val);
  } else {
    Serial.printf("rxCount = %d", rxCount);
  }
}


      // Begin test
      // Dump all the registers from 0x00 to 0xFF
      if ((char)payload[0] == 'D')
      {
        for (char t = 0x00; t < 0xFF; t++)
        {
          Serial.printf("%01X -> ", t);
          getDBG(t);
          Serial.println();
          delay(400);
        }
      }
  
      // Read one register from payload
      if ((char)payload[0] == 'R')
      {
        char t[] = {(char)payload[1],(char)payload[2]};
        t[2] = '\0';
        char num = (char)strtol(t, NULL, 16);
        getDBG(num);
      }
  
      // Write one register from payload
      if ((char)payload[0] == 'X')
      {
        char mem = (char)payload[1];
        char t[] = {(char)payload[2],(char)payload[3]};
        t[2] = '\0';
        char num = (char)strtol(t, NULL, 16);
        char z[] = {(char)payload[4],(char)payload[5]};
        z[2] = '\0';
        char test = (char)strtol(z, NULL, 16);
        char fixed;
        if (mem == 'R')
          fixed = 0x80;
        else
          fixed = 0xA0;
  
        char chksum = fixed+num+test;
        char writeReg[4] = {fixed, num, test, chksum};
        Serial.printf("%01X %01X %01X %01X", writeReg[0], writeReg[1], writeReg[2], writeReg[3]);
        Serial.println();
        for (int i = 0; i < 4; i++)
        {
            StoveSerial.write(writeReg[i]);
            delay(1);
        }
      }
  
      // end test