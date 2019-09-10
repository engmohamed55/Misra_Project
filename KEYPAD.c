/*
 * KEYPAD.c
 *
 * Created: 30/01/2016 06:38:37 م
 *  Author: hossam
 */ 
#include "DIO.h"
#include "KEYPAD.h"

/*Local Symbols*/



void KPD_COL_INIT(void)
{
    DIO_vidWritePortDirection(KPD_COL_PORT,KPD_COL_MASK,(u8)0x00);
}

void KPD_ROW_INIT(void)
{
    DIO_vidWritePortDirection(KPD_ROW_PORT,KPD_ROW_MASK,(unsigned char)0xff);
    DIO_vidWritePortData(KPD_ROW_PORT,KPD_ROW_MASK,(unsigned char)0x00);
}

void KPD_ROW_WRITE(unsigned char DATA)
{
    DIO_vidWritePortData(KPD_ROW_PORT,KPD_ROW_MASK,((DATA) << KPD_ROW_PIN_NUM));
}



void KPD_COL_READ(unsigned char* VALPTR)
{
    DIO_vidReadPortData(KPD_COL_PORT,KPD_COL_MASK,(VALPTR));
    (*(VALPTR) = (*(VALPTR)) >> KPD_COL_PIN_NUM);
}

void KPD_Init(void)
{
	KPD_COL_INIT();
	KPD_ROW_INIT();
	
}
void KPD_ReadVal(char* ValuePtr)
{
    static const char KeysLut[]= {'1' , '2' , '3' , '4' , '5' , '6','7' , '8' , '9','*' , '0' , '#'};
	unsigned char Rowdata;
	unsigned char ColData;
	unsigned char LoopTermnate = 0;
	for(Rowdata = (unsigned char)0 ; (Rowdata < (unsigned char)4) && (LoopTermnate == (unsigned char)0) ; Rowdata ++)
	{
		KPD_ROW_WRITE(((unsigned char)1<<Rowdata));
		KPD_COL_READ(&ColData);

		if(ColData != (unsigned char)0)
		{
			*ValuePtr = KeysLut[(Rowdata*(unsigned char)3) + (ColData/(unsigned char)2)];
			LoopTermnate = (unsigned char)1;
		}
		else
		{
			*ValuePtr = 'n';
		}
	}

	
	
}

