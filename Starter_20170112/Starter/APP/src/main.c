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
    u16 i;
    u32 j = 0;
    FATFS fs;            // Work area (file system object) for logical drive
    FATFS *fss;
    FIL fsrc;            // 文件结构

    DIR DirObject;       //目录结构

	BSP_Init();
	//MUX_Select(3); // ME2(3) PMS(1)
	SPI_SD_Configuration();

	  printf("start to read file\n");

	  /* Register volume work area (never fails) */
	  f_mount(0, &fatfs);

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

/*
    for(i=0;i<1024;i++)
      send_buffer[i]='c';
    printf("\r\n");

    // 为SD卡开辟一个工作区
    j = f_mount(0, &fs);
    printf("j is %d \r\n", j);

    //correct till here//


       res=f_getfree("/",&free_clust,&fss);

       if(res==FR_OK)
       {
         //printf("该分区所有扇区数为：%d\r\n",(fss->max_clust-2)*(fss->csize));
         //printf("该分区空簇数为：%d\r\n",free_clust);
         //printf("该分区空扇区数为：%d\r\n",free_clust*(fss->csize));
         printf("total：%d\r\n",(fss->max_clust-2)*(fss->csize));
         printf("kong su number：%d\r\n",free_clust);
         printf("kong shuan number：%d\r\n",free_clust*(fss->csize));       }
       else
    	 //printf("获取分区空簇失败\r\n");
       	 printf("partition failure\r\n");

       res=f_open(&fsrc,"/test1.txt",FA_CREATE_NEW | FA_WRITE ); //create a new file
       printf("\r\n");

       if(res==FR_OK)
       {
         printf("创建文件或打开文件成功  O(∩_∩)O\r\n");
         printf("该文件属性:%d\r\n",fsrc.flag);
         printf("该文件大小：%d\r\n",fsrc.fsize);
         printf("该文件读写开始处：%d\r\n",fsrc.fptr);
         printf("该文件开始簇号:%d\r\n",fsrc.org_clust);
         printf("该文件当前簇号：%d\r\n",fsrc.curr_clust);
         printf("该文件当前扇区号:%d\r\n",fsrc.dsect);
       }
       else if(res==FR_EXIST)
         printf("already exists\r\n");
       else
           //printf("创建文件或打开文件失败~~~~(>_<)~~~~ \r\n");
       	   printf("create or open file fail~~~~(>_<)~~~~ \r\n");

       f_close(&fsrc);//关闭文件
       res=f_open(&fsrc,"/test1.txt", FA_WRITE ); // open a file
       printf("res =%d\r\n", res);

       //correct until here.

       printf("\nCreate a new file (hello.txt).\n");
       res = f_open(&fsrc, "HELLO.TXT", FA_WRITE | FA_CREATE_ALWAYS);
       printf("res =%d\r\n", res);

     	printf("\nWrite a text data. (Hello world!)\n");
     	res = f_write(&fsrc, "Hello world!\r\n", 14, &bw);
        printf("res =%d\r\n", res);
     	printf("%u bytes written.\n", bw);

       send_buffer[0] = 'A';
       send_buffer[1] = 'B';
       send_buffer[2] = 'C';

       res=f_write(&fsrc,send_buffer,sizeof(send_buffer),&bw);//向/test1/test.c文件中写入1024字节数据
       printf("res is %d\r\n", res);
       f_close(&fsrc);//关闭文件
        if(res==FR_OK)
          printf("Write successful:%dBytes\r\n",bw);
        else
          printf("write fails~~~~(>_<)~~~~ \r\n");

        if(bw<24)
          printf("memory overflow\r\n");

        f_open(&fsrc, "/test1.txt", FA_OPEN_EXISTING | FA_READ);

       res=f_read(&fsrc,receive_buffer,1024,&br);

       if(res==FR_OK)
         printf("read successful：%dBytes\r\n",br);
       else
         printf("read fail\r\n");
       if(br<1024)
         printf("end of file\r\n");

       f_close(&fsrc);//关闭文件

       // 注销工作区
       f_mount(0, NULL);
*/


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
  //无线循环
	for (;;) ;
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
