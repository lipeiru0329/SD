#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "xprintf.h"
#include "BSP.h"
#include "am2301.h"
#include "scheduler.h"
#include "oled.h"
#include "diskio.h"
#include "ff.h"
#include "mmc_sd.h"

//#define _FS_READONLY  0

const uint8_t PMS5003S_port = 2;
char data[32];
int ME2_counter = 0;
int ME2_CollectDataFlag= 0;
int PMS_counter = 0;
int PMS_CollectDataFlag= 0;
int newdata_flag = 0;
int muxport = 1; //ME2 = 1; PMS = 2;
int ME2_dataID = 0;
int PMS_dataID = 0;
TM_AM2301_Data_t tempData;

uint8_t Retry = 0;

DSTATUS status;

int retry = 0;
int r1 = 2;
//int r9 =0;

/***************************/
//unsigned char send_buffer[1024];    //发送数据缓冲区
char send_buffer[1024];    //发送数据缓冲区
unsigned char receive_buffer[1024];//接收数据缓冲区


unsigned char Buffercmp(u8* pBuffer1, u8* pBuffer2, u16 BufferLength);
FRESULT res;         // FatFs 中函数执行结果
UINT bw,br;          // 写入和读出数据字节计数器
DWORD free_clust;    //空簇，空扇区大小

FRESULT rc;				/* Result code */
	FATFS fatfs;			/* File system object */
	FIL fil;					/* File object */
	DIR dir;					/* Directory object */
	FILINFO fno;			/* File information object */
	UINT bw, br, i;
	BYTE buff[128];
/* Private function prototypes -----------------------------------------------*/
void die (FRESULT rc);


void SPI_SD_Configuration(void);

int main()
{

	BSP_Init();
	SPI_SD_Configuration();

	printf("start to read file\n");

	/* Register volume work area (never fails) */
	f_mount(0, &fatfs);

	printf("\nOpen a test file (test.txt).\n");
	rc = f_open(&fil, "test.txt", FA_READ);
	if (rc) die(rc);

	printf("\nType the file content.\n");
	for (;;) {
		rc = f_read(&fil, buff, sizeof(buff), &br);	/* Read a chunk of file */
		if (rc || !br) break;			/* Error or end of file */
		for (i = 0; i < br; i++)		/* Type the data */
			putchar(buff[i]);
	}
	if (rc) die(rc);

	printf("\nClose the file.\n");
	rc = f_close(&fil);
	if (rc) die(rc);


	printf("\nCreate a new file (hello.txt).\n");
	rc = f_open(&fil, "HELLO.TXT", FA_WRITE | FA_CREATE_ALWAYS);
	if (rc) die(rc);

	printf("\nWrite a text data. (Hello world!)\n");
	rc = f_write(&fil, "Hello world!\r\n", 14, &bw);
	if (rc) die(rc);
	printf("%u bytes written.\n", bw);

	printf("\nClose the file.\n");
	rc = f_close(&fil);
	if (rc) die(rc);

}

void SPI_SD_Configuration(void)
{      // SPI_ControLine();
	  SPI_InitTypeDef  SPI_InitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;


	  RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1 |RCC_APB2Periph_GPIOA, ENABLE);

	  /* NSS---->GPIO(SD) */
	  SPI_SSOutputCmd(SPI1, ENABLE);
	  /* CLK n MOSI  */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure); // PORTA PIN

	  /* MISO */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU ; // GPIO_Mode_IPU ?? GPIO_Mode_Out_PP
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* CS */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);


	  /* SPI1 Config // SPI_setspeed(HIGH);-----------------------------------------------*/
	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	  SPI_InitStructure.SPI_CRCPolynomial = 7;
	  SPI_Init(SPI1, &SPI_InitStructure);
	  /* Enable SPI1 */
	  SPI_Cmd(SPI1, ENABLE);
}


unsigned char Buffercmp(u8* pBuffer1, u8* pBuffer2, u16 BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return 0;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return 1;
}

void die (FRESULT rc)
{
	printf("Failed with rc=%u.\n", rc);
	for (;;)
		;
}

DWORD get_fattime (void)
{
	return	  ((DWORD)(2010 - 1980) << 25)	/* Fixed to Jan. 1, 2010 */
			| ((DWORD)1 << 21)
			| ((DWORD)1 << 16)
			| ((DWORD)0 << 11)
			| ((DWORD)0 << 5)
			| ((DWORD)0 >> 1);
}
