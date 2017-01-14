#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "xprintf.h"
#include "diskio.h"
#include "ff.h"
#include "mmc_sd.h"
#include "stdio.h"

FRESULT rc;				/* Result code */
	FATFS fatfs;			/* File system object */
	FIL fil;					/* File object */
	DIR dir;					/* Directory object */
	FILINFO fno;			/* File information object */
	UINT bw, br, i;
	BYTE buff[128];

#define MAXLENGTH 100

char data[]="today is a nice day\n";
int length = 0;

/* Private function prototypes -----------------------------------------------*/
void die (FRESULT rc);
void SPI_SD_Configuration(void);
void write_new_data(char* data, uint8_t length);

int main()
{

	SPI_SD_Configuration();

	write_new_data(data, sizeof(data));

	f_mount(0, &fatfs);

	printf("\nOpen a test file (test.txt).\n");
	rc = f_open(&fil, "data.csv", FA_READ);
	if (rc) die(rc);

	printf("\nType the file content.\n");
	for (;;) {
		rc = f_read(&fil, buff, sizeof(buff), &br);
		if (rc || !br) break;
		for (i = 0; i < br; i++)
			putchar(buff[i]);
	}
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

void write_new_data(char new_str[], uint8_t length)
{
	printf("start to read file\n");

	/* Register volume work area (never fails) */
	f_mount(0, &fatfs);

	printf("\nOpen the data file (data.csv).\n");
	rc = f_open(&fil, "data.csv", FA_WRITE | FA_CREATE_ALWAYS);
	if (rc) die(rc);

	printf("\nWrite a text data. (Hello world!)\n");
	rc = f_write(&fil, new_str, length, &bw);
	if (rc) die(rc);
	printf("%u bytes written.\n", bw);

	printf("\nClose the file.\n");
	rc = f_close(&fil);
	if (rc) die(rc);

}
