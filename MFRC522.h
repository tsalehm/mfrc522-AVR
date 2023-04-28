#ifndef _DELAY_INCLUDED_
#include <delay.h>
#endif
//    AVR  --  MFRC522
//   MISO  <-  MISO
//   MOSI  ->  MOSI
//    SCK  ->  SCK
//PORTB.1  ->  SDA

////////////   PORTB.2 is ss (sda)(nss) pin
#define cs_set PORTB.0=1
#define cs_clr PORTB.0=0


#pragma used+

/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH       64                 //FIFO size=64byte
#define MAXRLEN               18

/////////////////////////////////////////////////////////////////////
#define PCD_IDLE              0x00               //取消当前命令
#define PCD_AUTHENT           0x0E               //验证密钥
#define PCD_RECEIVE           0x08               //接收数据
#define PCD_TRANSMIT          0x04               //发送数据
#define PCD_TRANSCEIVE        0x0C               //发送并接收数据
#define PCD_RESETPHASE        0x0F               //复位
#define PCD_CALCCRC           0x03               //CRC计算

/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL           0x26               //寻天线区内未进入休眠状态
#define PICC_REQALL           0x52               //寻天线区内全部卡
#define PICC_ANTICOLL1        0x93               //防冲撞
#define PICC_ANTICOLL2        0x95               //防冲撞
#define PICC_AUTHENT1A        0x60               //验证A密钥
#define PICC_AUTHENT1B        0x61               //验证B密钥
#define PICC_READ             0x30               //读块
#define PICC_WRITE            0xA0               //写块
#define PICC_DECREMENT        0xC0               //扣款
#define PICC_INCREMENT        0xC1               //充值
#define PICC_RESTORE          0xC2               //调块数据到缓冲区
#define PICC_TRANSFER         0xB0               //保存缓冲区中数据
#define PICC_HALT             0x50               //休眠

/////////////////////////////////////////////////////////////////////
// PAGE 0
#define     RFU00                 0x00
#define     CommandReg            0x01
#define     ComIEnReg             0x02
#define     DivlEnReg             0x03
#define     ComIrqReg             0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     RFU0F                 0x0F
// PAGE 1
#define     RFU10                 0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     RFU1A                 0x1A
#define     RFU1B                 0x1B
#define     MifareReg             0x1C
#define     RFU1D                 0x1D
#define     RFU1E                 0x1E
#define     SerialSpeedReg        0x1F
// PAGE 2
#define     RFU20                 0x20
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     RFU23                 0x23
#define     ModWidthReg           0x24
#define     RFU25                 0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsCfgReg            0x28
#define     ModGsCfgReg           0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
// PAGE 3
#define     RFU30                 0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     RFU3C                 0x3C
#define     RFU3D                 0x3D
#define     RFU3E                 0x3E
#define     RFU3F		  0x3F

/////////////////////////////////////////////////////////////////////
#define MI_OK                          0
#define MI_NOTAGERR                    (-1)
#define MI_ERR                         (-2)

/////////////////////////////////////////////////////////////////////
char PcdReset(void);
void PcdAntennaOn(void);
void PcdAntennaOff(void);
char PcdRequest(unsigned char req_code,unsigned char *pTagType);
char PcdAnticoll(unsigned char *pSnr);
char PcdSelect(unsigned char *pSnr);
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr);
char PcdRead(unsigned char addr,unsigned char *pData);
char PcdWrite(unsigned char addr,unsigned char *pData);
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue);
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr);
char PcdHalt(void);
char PcdComMF522(unsigned char Command,
                 unsigned char *pInData,
                 unsigned char InLenByte,
                 unsigned char *pOutData,
                 unsigned int  *pOutLenBit);
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData);
///////////////////////////////////////////////////////////////////
void WriteRawRC(unsigned char reg,unsigned char value);
unsigned char ReadRawRC(unsigned char reg);
void SetBitMask(unsigned char reg,unsigned char mask);
void ClearBitMask(unsigned char reg,unsigned char mask);

///////////////////////////////////////////////////////////////

char PcdReset(void)
{
	//unsigned char i;
 /*   MF522_RST=1;

		_nop_();

    MF522_RST=0;

		_nop_();

    MF522_RST=1;

		_nop_();    */

    WriteRawRC(CommandReg,PCD_RESETPHASE);

	//	_nop_();


    WriteRawRC(ModeReg,0x3D);            //和Mifare卡通讯，CRC初始值0x6363
    WriteRawRC(TReloadRegL,30);
    WriteRawRC(TReloadRegH,0);
    WriteRawRC(TModeReg,0x8D);
    WriteRawRC(TPrescalerReg,0x3E);
    WriteRawRC(TxAutoReg,0x40);

    return MI_OK;
}

void PcdAntennaOn()
{
    unsigned char i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}

void PcdAntennaOff()
{
    ClearBitMask(TxControlReg, 0x03);
}

char PcdRequest(unsigned char req_code,unsigned char *pTagType)
{
/* Function Name:MFRC522_Request
 * Description: Find cards, read the card type number
 * Input parameters: reqMode - find cards way
 *   TagType - Return Card Type
 *    0x4400 = Mifare_UltraLight
 *    0x0400 = Mifare_One(S50)
 *    0x0200 = Mifare_One(S70)
 *    0x0800 = Mifare_Pro(X)
 *    0x4403 = Mifare_DESFire
 * Return value: the successful return MI_OK */

   char status;
   unsigned int  unLen;
   unsigned char ucComMF522Buf[MAXRLEN];

   ClearBitMask(Status2Reg,0x08);
   WriteRawRC(BitFramingReg,0x07);
   SetBitMask(TxControlReg,0x03);

   ucComMF522Buf[0] = req_code;

   status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);

   if ((status == MI_OK) && (unLen == 0x10))
   {
       *pTagType     = ucComMF522Buf[0];
       *(pTagType+1) = ucComMF522Buf[1];
   }
   else
   {
        status = MI_ERR;
   }

   return status;
}




char PcdAnticoll(unsigned char *pSnr)
{
 /* Function Name: MFRC522_Anticoll
 * Description: Anti-collision detection, reading selected card serial number card
 * Input parameters: serNum - returns 4 bytes card serial number, the first 5 bytes for the checksum byte
 * Return value: the successful return MI_OK */

    char status;
    unsigned char i,snr_check=0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];


    ClearBitMask(Status2Reg,0x08);
    WriteRawRC(BitFramingReg,0x00);
    ClearBitMask(CollReg,0x80);

    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    if (status == MI_OK)
    {
    	 for (i=0; i<4; i++)
         {
             *(pSnr+i)  = ucComMF522Buf[i];
             snr_check ^= ucComMF522Buf[i];

         }
         if (snr_check != ucComMF522Buf[i])
         {   status = MI_ERR;    }
    }

    SetBitMask(CollReg,0x80);

    return status;
}

char PcdSelect(unsigned char *pSnr)
{
/*
 * Function Name: MFRC522_SelectTag
 * Description: election card, read the card memory capacity
 * Input parameters: serNum - Incoming card serial number
 * Return value: the successful return of card capacity
 */
    char status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);

    ClearBitMask(Status2Reg,0x08);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);

    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
 //   memcpy(&ucComMF522Buf[2], pKey, 6);
 //   memcpy(&ucComMF522Buf[8], pSnr, 4);

    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }

    return status;
}

char PcdRead(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))
 //   {   memcpy(pData, ucComMF522Buf, 16);   }
    {
        for (i=0; i<16; i++)
        {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }

    return status;
}

char PcdWrite(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    if (status == MI_OK)
    {
        //memcpy(ucComMF522Buf, pData, 16);
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }

    return status;
}

char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    if (status == MI_OK)
    {
       // memcpy(ucComMF522Buf, pValue, 4);
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pValue+i);   }
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
        unLen = 0;
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }

    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    return status;
}

char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    if (status == MI_OK)
    {
        ucComMF522Buf[0] = 0;
        ucComMF522Buf[1] = 0;
        ucComMF522Buf[2] = 0;
        ucComMF522Buf[3] = 0;
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }

    if (status != MI_OK)
    {    return MI_ERR;   }

    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    return status;
}

char PcdHalt(void)
{
//    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    //status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return MI_OK;
}

char PcdComMF522(unsigned char Command,
                 unsigned char *pInData,
                 unsigned char InLenByte,
                 unsigned char *pOutData,
                 unsigned int  *pOutLenBit)
{
    char status = MI_ERR;
    unsigned char irqEn   = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    unsigned int i;
    switch (Command)
    {
       case PCD_AUTHENT:
          irqEn   = 0x12;
          waitFor = 0x10;
          break;
       case PCD_TRANSCEIVE:
          irqEn   = 0x77;
          waitFor = 0x30;
          break;
       default:
         break;
    }

    WriteRawRC(ComIEnReg,irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);

    for (i=0; i<InLenByte; i++)
    {   WriteRawRC(FIFODataReg, pInData[i]);    }
    WriteRawRC(CommandReg, Command);


    if (Command == PCD_TRANSCEIVE)
    {    SetBitMask(BitFramingReg,0x80);  }

    i = 600;      //根据时钟频率调整，操作M1卡最大等待时间25ms
    do
    {
         n = ReadRawRC(ComIrqReg);
         i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));
    ClearBitMask(BitFramingReg,0x80);

    if (i!=0)
    {
         if(!(ReadRawRC(ErrorReg)&0x1B))
         {
             status = MI_OK;
             if (n & irqEn & 0x01)
             {   status = MI_NOTAGERR;   }
             if (Command == PCD_TRANSCEIVE)
             {
               	n = ReadRawRC(FIFOLevelReg);
              	lastBits = ReadRawRC(ControlReg) & 0x07;
                if (lastBits)
                {   *pOutLenBit = (n-1)*8 + lastBits;   }
                else
                {   *pOutLenBit = n*8;   }
                if (n == 0)
                {   n = 1;    }
                if (n > MAXRLEN)
                {   n = MAXRLEN;   }
                for (i=0; i<n; i++)
                {   pOutData[i] = ReadRawRC(FIFODataReg);    }
            }
         }
         else
         {   status = MI_ERR;   }
   }


   SetBitMask(ControlReg,0x80);           // stop timer now
   WriteRawRC(CommandReg,PCD_IDLE);

   return status;
}

void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   WriteRawRC(FIFODataReg, *(pIndata+i));   }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
}

/////////////////////////////////////////////////////////
void WriteRawRC(unsigned char reg,unsigned char value)
{
    cs_clr;
    spi(reg<<1);
    spi(value);
    cs_set;
}

unsigned char ReadRawRC(unsigned char reg)
{
    cs_clr;
    spi(0x80 | (reg<<1));
    reg = spi(0);
    cs_set;
    return reg;
}

void SetBitMask(unsigned char reg,unsigned char mask)
{
    unsigned char tmp;
    tmp = ReadRawRC(reg);
    tmp |= mask;
    WriteRawRC(reg,tmp);
}

void ClearBitMask(unsigned char reg,unsigned char mask)
{
    unsigned char tmp;
    tmp = ReadRawRC(reg);
    tmp &= (~mask);
    WriteRawRC(reg,tmp);
}


////////////////////////////////////////////////////////////////

unsigned char rd_trailer(unsigned char sector_num,unsigned char keyA[6],unsigned char keyB[6],unsigned char access_keys[3],unsigned char secret_data,unsigned char key[6]){
   unsigned char dd[20],stat,k;
   //Read data only from Trailer blocks
   stat = PcdRequest(PICC_REQALL, dd); // Type of mod.
   if(!stat){   
      stat = PcdAnticoll(dd); //Serial number of selected type ...
      stat = PcdSelect(dd);   //Select
      stat = PcdAuthState(PICC_AUTHENT1A, ((sector_num+1)*4)-1, key, dd); //Send A key
      stat = PcdRead(((sector_num+1)*4)-1, dd); //Read data[4] at Address Addr

      PcdReset();
      delay_ms(1);
      PcdAntennaOff();
      delay_ms(1);
      PcdAntennaOn();
      delay_ms(1);
        PcdHalt();
      
      for(k=0;k<6;k++)
         keyA[k]=dd[k]; 
         
      for(k=0;k<3;k++)
         access_keys[k]=dd[k+6]; 
         
         secret_data=dd[9];
         
      for(k=0;k<6;k++)
         keyB[k]=dd[k+10];   
         
      if(stat==MI_OK){
         return 1;
      }else{
         return 0;
      }
   }else{
      PcdReset();
      delay_ms(1);
      PcdAntennaOff();
      delay_ms(1);
      PcdAntennaOn();
      delay_ms(1);
        PcdHalt();
      return 0; 
   }
}
unsigned char wr_trailer(unsigned char sector_num,unsigned char keyA[6],unsigned char keyB[6],unsigned char access_keys[3],unsigned char secret_data,unsigned char key[6]){
   unsigned char dd[20],stat,k;
   //Write Keys only to Trailer blocks with all access to all Block in this Sector
   stat = PcdRequest(PICC_REQALL, dd); // Type of mod. 
   if(!stat){
      stat = PcdAnticoll(dd); //Serial number of selected type ...
      stat = PcdSelect(dd);   //Select
      stat = PcdAuthState(PICC_AUTHENT1A, ((sector_num+1)*4)-1 , key, dd); //Send A key

      for(k=0;k<6;k++)dd[k]=keyA[k];
      dd[9]=secret_data;
      for(k=0;k<6;k++)dd[k+10]=keyB[k];  
       for(k=0;k<3;k++)dd[k+6]=access_keys[k];
      stat = PcdWrite(((sector_num+1)*4)-1 , dd); //Write data[4] at Address Addr

      PcdReset();
      delay_ms(1);
      PcdAntennaOff();
      delay_ms(1);
      PcdAntennaOn();
      delay_ms(1);
        PcdHalt();

      if(stat==MI_OK){
         return 1;
      }else{
         return 0;
      }
   }else{
      PcdReset();
      delay_ms(1);
      PcdAntennaOff();
      delay_ms(1);
      PcdAntennaOn();
      delay_ms(1);
        PcdHalt();
      return 0; 
   }
}
unsigned char wr_data(unsigned char wrdata[16],unsigned char sector_num, unsigned char block_num, unsigned char key[6]){
   unsigned char dd[20],stat,address,k;
   //Write data to only value blocks
   //write to trailer sectors will be ignored
   address=(((sector_num+1)*4)-4)+block_num;
   if((address+1)%4){
      stat = PcdRequest(PICC_REQALL, dd); // Type of mod. 
      if(!stat){
         stat = PcdAnticoll(dd); //Serial number of selected type ...
         stat = PcdSelect(dd);   //Select
         stat = PcdAuthState(PICC_AUTHENT1A, address, key, dd); //Send A key

         for(k=0;k<16;k++)dd[k]=wrdata[k];
         stat = PcdWrite(address, dd); //Write data[4] at Address Addr

         PcdReset();
         delay_ms(1);
         PcdAntennaOff();
         delay_ms(1);
         PcdAntennaOn();
         delay_ms(1);
           PcdHalt();

         if(stat==MI_OK){
            return 1;
         }else{
            return 0;
         }
      }else{
         PcdReset();
         delay_ms(1);
         PcdAntennaOff();
         delay_ms(1);
         PcdAntennaOn();
         delay_ms(1);
           PcdHalt();
         return 0; 
      }
   }else{     
      return 0;
   }
}
unsigned char rd_data(unsigned char rddata[16],unsigned char sector_num, unsigned char block_num, unsigned char key[6]){
   unsigned char dd[20],flag_err,stat,k,address;
   //Read data only from only blocks
   //Read from trailer sectors will be ignored
  address=(((sector_num+1)*4)-4)+block_num;
  if((address+1)%4){
      stat = PcdRequest(PICC_REQALL, dd); // Type of mod.
      if(!stat){   
         stat = PcdAnticoll(dd); //Serial number of selected type ...
         stat = PcdSelect(dd);   //Select
         stat = PcdAuthState(PICC_AUTHENT1A, address, key, dd); //Send A key
         stat = PcdRead(address, dd); //Read data[4] at Address Addr

         PcdReset();
         delay_ms(1);
         PcdAntennaOff();
         delay_ms(1);
         PcdAntennaOn();
         delay_ms(1);
           PcdHalt();

         for(k=0;k<16;k++)rddata[k]=dd[k];
         if(stat==MI_OK){
            return 1;
         }else{
            return 0;
         }
      }else{
         PcdReset();
         delay_ms(1);
         PcdAntennaOff();
         delay_ms(1);
         PcdAntennaOn();
         delay_ms(1);
           PcdHalt();
         return 0; 
      }
   }else{
      return 0;
   }
}
unsigned char rd_serial(unsigned char serial[4]){
unsigned char dd[20],stat;
   stat = PcdRequest(PICC_REQALL, dd); // Type of mod.
   if(!stat){   
      stat = PcdAnticoll(dd); //Serial number of selected type ...
        PcdHalt();
      serial[0]=dd[0];
      serial[1]=dd[1];
      serial[2]=dd[2];
      serial[3]=dd[3];
      if(stat==MI_OK){
         return 1;
      }else{
         return 0;
      }
   }else{
      return 0;
   }
}
void reset_mfrc(void){
   PcdReset();
   delay_ms(1);
   PcdAntennaOff();
   delay_ms(1);
   PcdAntennaOn();
   delay_ms(1);
     PcdHalt();
}

#pragma used-

/*
unsigned char wr_data(unsigned char wrdata[4],unsigned char sector_num, unsigned char block_num, unsigned char key[6]){
   unsigned char dd[20],stat,address;
   //Write data to only value blocks
   //write to trailer sectors will be ignored
   address=(((sector_num+1)*4)-4)+block_num;
   if((address+1)%4){
      stat = PcdRequest(PICC_REQALL, dd); // Type of mod. 
      if(!stat){
         stat = PcdAnticoll(dd); //Serial number of selected type ...
         stat = PcdSelect(dd);   //Select
         stat = PcdAuthState(PICC_AUTHENT1A, address, key, dd); //Send A key

         dd[0]=wrdata[0]; dd[1]=wrdata[1]; dd[2]=wrdata[2]; dd[3]=wrdata[3];
         dd[4]=255-wrdata[0];dd[5]=255-wrdata[1];dd[6]=255-wrdata[2];dd[7]=255-wrdata[3];
         dd[8]=wrdata[0]; dd[9]=wrdata[1]; dd[10]=wrdata[2];dd[11]=wrdata[3];
         dd[12]=address;   dd[13]=255-address;  dd[14]=address;   dd[15]=255-address;
         stat = PcdWrite(address, dd); //Write data[4] at Address Addr

         PcdReset();
         delay_ms(1);
         PcdAntennaOff();
         delay_ms(1);
         PcdAntennaOn();
         delay_ms(1);
           PcdHalt();

         if(stat==MI_OK){
            return 1;
         }else{
            return 0;
         }
      }else{
         PcdReset();
         delay_ms(1);
         PcdAntennaOff();
         delay_ms(1);
         PcdAntennaOn();
         delay_ms(1);
           PcdHalt();
         return 0; 
      }
   }else{     
      return 0;
   }
}
unsigned char rd_data(unsigned char rddata[4],unsigned char sector_num, unsigned char block_num, unsigned char key[6]){
   unsigned char dd[20],flag_err,stat,k,address;
   //Read data only from only blocks
   //Read from trailer sectors will be ignored
  address=(((sector_num+1)*4)-4)+block_num;
  if((address+1)%4){
      stat = PcdRequest(PICC_REQALL, dd); // Type of mod.
      if(!stat){   
         stat = PcdAnticoll(dd); //Serial number of selected type ...
         stat = PcdSelect(dd);   //Select
         stat = PcdAuthState(PICC_AUTHENT1A, address, key, dd); //Send A key
         stat = PcdRead(address, dd); //Read data[4] at Address Addr

         PcdReset();
         delay_ms(1);
         PcdAntennaOff();
         delay_ms(1);
         PcdAntennaOn();
         delay_ms(1);
           PcdHalt();

         flag_err=0;
         for(k=0;k<4;k++){
            if(!(dd[k]==dd[k+8] && dd[k]==(255-dd[k+4]))){
               flag_err=1;
            }
         }
         if(!flag_err){
            rddata[0]=dd[0];
            rddata[1]=dd[1];
            rddata[2]=dd[2];
            rddata[3]=dd[3];
              PcdHalt();
            if(stat==MI_OK){
               return 1;
            }else{
               return 0;
            }
         }else{
            return 0;
         }
      }else{
         PcdReset();
         delay_ms(1);
         PcdAntennaOff();
         delay_ms(1);
         PcdAntennaOn();
         delay_ms(1);
           PcdHalt();
         return 0; 
      }
   }else{
      return 0;
   }
}
*/
