#include "Basic_Types.h"
#include "CRC.h"

/***************************************************************************************/
static u32 GetPower(u32 Base,u32 Pow);
static u16 gen_crc16(const u8 data[], u16 size, u32 CRC16);
u32 randi(void);
/***************************************************************************************/
void SECR_CrcPolynomialGenerate(u32* PolynomialPtr,u8 CrcLengthInBits)
{
	u32 DevisorValue;
	DevisorValue = (GetPower((u32)2,(u32)CrcLengthInBits)) - (u32)1;
	*PolynomialPtr = ((u32)randi() % (u32)DevisorValue) +(u32)0x10000 ;
}
/***************************************************************************************/
void SECR_GnerateCrc(const u8 PayloadPtr[],u16 PayloadLength, u16* CrcPtr, u32 CrcPoly)
{
	u16 LoopIndex,p;
	static u8 InternalBuffer[8];
	/*Copying data to internal buffer*/
	for (LoopIndex = (u8)0; LoopIndex < PayloadLength; LoopIndex ++)
	{
	    p = LoopIndex;
		InternalBuffer[LoopIndex] = PayloadPtr[LoopIndex];
	}
	/*perform bit wise invert on the data*/
	for (LoopIndex = (u8)0; LoopIndex < PayloadLength; LoopIndex ++)
	{
		InternalBuffer[LoopIndex]  ^= (u8)0xff;
	}
	/*Generate CRC*/
	*CrcPtr = gen_crc16(InternalBuffer,(u16)PayloadLength*(u16)8,(u32)0x18005);
}
/***************************************************************************************/
static u32 GetPower(u32 Base,u32 Pow)
{
	u32 result = (u8)1;
	u32 LoopIndex2;
	for (LoopIndex2 = (u8)0; LoopIndex2 < Pow; LoopIndex2 ++)
	{
		result *= Base;
	}
	return result;
}
/***************************************************************************************/
static u16 gen_crc16(const u8 data[], u16 size, u32 CRC16)
{
    u16 m,test;
	u16 out = 0;
	u16 bits_read = 0, bit_flag;
	u16 i;
	u16 crc = (u16)0;
	u16 j = (u16)0x0001;
	/* Sanity check: */
	if(data == (u8)0)
	{
	    m = (u16)0;
	}
	while(size > (u16)0)
	{
		bit_flag = out >> (u16)15;

		/* Get next bit: */
		out <<= (u16)1;
		test = (*data >> bits_read) & (u16)1; /*item a) work from the least significant bits*/
		out |= test;
		/* Increment bit counter: */
		bits_read++;
		if(bits_read > (u16)7)
		{
			bits_read = (u16)0;
			size--;
		}

		/* Cycle check: */
		if(bit_flag)
		{
		out ^= CRC16;
		}

	}

	/*item b) "push out" the last 16 bits*/

	for (i = (u16)0; i < (u16)16; ++i) {
		bit_flag = out >> (u16)15;
		out <<= (u16)1;
		if(bit_flag)
		{
		out ^= CRC16;
		}
		}

	/*item c) reverse the bits*/

	i = (u16)0x8000;

	for (; i != (u16)0; i >>=1 ) {

		if (i && out)
		    {
		    crc |= j;
		    }
		 j <<= 1;
	}

	return crc;
}
/***************************************************************************************/
