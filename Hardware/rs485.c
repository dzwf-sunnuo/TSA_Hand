#include "rs485.h"
#include "usart.h"
#include "stdio.h"
#include "wrist.h"
extern volatile uint8_t wrist_enabled;  // 手腕运动使能标志
extern volatile float motion_speed_factor;
extern volatile uint16_t cycle_time;
//第1套连续动作：数数字0~9，包含11个手势
  static uint16_t Action1_ban1[12][5]={
										 {0x0064,0x0064,0x0064,0x0000,0x0101},//舒展
									   {0x1932,0x2032,0x1720,0x0000,0x0101},//“1”-伸食指
									   {0x1932,0x2032,0x1720,0x0000,0x0101},//“2”-伸中指
									   {0x1932,0x2032,0x1720,0x0000,0x0101},//“3”-伸无名指
									   {0x1932,0x2032,0x1720,0x0000,0x0101},//“4”-伸小指
									   {0x0064,0x0064,0x0064,0x0000,0x0101},//“5”-舒展
									   {0x0064,0x0064,0x0064,0x0000,0x0101},//“6”
									   {0x1932,0x2032,0x1720,0x0000,0x0101},//“7”
									   {0x0064,0x0064,0x0064,0x0000,0x0101},//“8”
									   {0x1932,0x2032,0x1E20,0x0000,0x0101},//“9”
									   {0x1932,0x2032,0x1E20,0x0000,0x0101},//“0”-握拳
									   {0x0064,0x0064,0x0064,0x0000,0x0101}};//舒展
  static uint16_t Action1_ban2[12][5]={
										 {0x0064,0x0064,0x0064,0x0064,0x0101},//舒展
									   {0x0064,0x2064,0x1E64,0x2764,0x0101},//“1”-伸食指
									   {0x0064,0x0064,0x1E64,0x2764,0x0101},//“2”-伸中指
									   {0x0064,0x0064,0x0064,0x2764,0x0101},//“3”-伸无名指
									   {0x0064,0x0064,0x0064,0x0064,0x0101},//“4”-伸小指
									   {0x0064,0x0064,0x0064,0x0064,0x0101},//“5”-舒展
									   {0x2864,0x2064,0x1E64,0x0064,0x0101},//“6”
									   {0x2064,0x1464,0x1F64,0x2764,0x0101},//“7”
									   {0x0064,0x2064,0x1F64,0x2764,0x0101},//“8”
									   {0x1964,0x2064,0x1F64,0x2764,0x0101},//“9”
									   {0x2A64,0x2064,0x1F64,0x2764,0x0101},//“0”-握拳
									   {0x0064,0x0064,0x0064,0x0064,0x0101}};//舒展
MODBUS modbus1;//定义MODBUS结构体类型的变量modbus1
MODBUS modbus2;//定义MODBUS结构体类型的变量modbus2

uint16_t Reg[100] = {0};//modbus2从机部分存储的数据，均是16位的数据
uint16_t Palm1[5] = {0};//存储发给从驱动板1的电机指令
uint16_t Palm2[5] = {0};//存储发给从驱动板2的电机指令
uint8_t i = 0;
//485串口发一个字节数据的函数
void modbus2_Send_Byte(uint8_t ch)
{
	/* 发送一个字节数据到USART2 */
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xff);	
}
void modbus1_Send_Byte(uint8_t ch)
{
	/* 发送一个字节数据到USART2 */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xff);	
}
//--------------------------------------------------串口2对应modbus2从机部分程序，与PC上位机相连----------------------------------------------------
// Modbus初始化函数，给定本机作为从机时的地址
void modbus2_Init()
{
	//为从机寄存器赋值，全局变量赋值必须在函数内进行！！！
	Reg[0] = 0x0000;//手腕电机1位置0~2000步【10==0x000A;1000==0x03E8;1990==0x0C76】
	Reg[1] = 0x0000;//手腕电机1速度0~4000步/s
	Reg[2] = 0x0000;//手腕电机2位置0~2000步【10==0x000A;1000==0x03E8;1990==0x0C76】
	Reg[3] = 0x0000;//手腕电机2速度0~4000步/s
	
	Reg[4] = 0x0000;//手指从驱动板1-M1——Palm1[0]
	Reg[5] = 0x0000;//手指从驱动板1-M2
	Reg[6] = 0x0000;//手指从驱动板1-M3
	Reg[7] = 0x0000;//手指从驱动板1-M4
	Reg[8] = 0x0000;//手指从驱动板1-标志位——Palm1[4]
	
	Reg[9] = 0x0000;//手指从驱动板2-M1——Palm2[0]
	Reg[10] = 0x0000;//手指从驱动板2-M2
	Reg[11] = 0x0000;//手指从驱动板2-M3
	Reg[12] = 0x0000;//手指从驱动板2-M4
	Reg[13] = 0x0000;//手指从驱动板2-标志位——Palm2[4]
	
	Reg[14] = 0x0000;//手腕运动标志位，高八位连续运动，第八位单独控制
	Reg[15] = 0x0320;//手腕运动速度控制1，总时间单位（ms）,默认800ms
	Reg[16] = 0x012C;//手腕起始位置，默认300，2.4cm
	Reg[17] = 0x06A4;//手腕最终位，默认1700，13.6cm
	
	modbus2.myadd = 0x10; //从机设备地址为16，与各个驱动板地址区分开来
	modbus2.timrun = 0;    //modbus2定时器停止计算
//	modbus2.slave_add=0x01;//主机要匹配的从机地址（本设备作为主机时）
}

// modbus2事件处理函数
void modbus2_Event()
{
	uint16_t crc,rccrc;//crc和接收到的crc
	//【1】没有收到数据包，return退出函数
	if(modbus2.reflag == 0){return;}//如果接收未完成则返回空，不做处理
	//【2】收到数据包(已经接收完成)：
	//先根据读到的数据帧做CRC校验：参数1是数组首地址，参数2是要计算的长度（除了CRC校验位其余全算）
	crc = Modbus_CRC16(&modbus2.rcbuf[0],modbus2.recount-2); //获取CRC校验位，校验函数与串口号无关
	rccrc = modbus2.rcbuf[modbus2.recount-2]*256+modbus2.rcbuf[modbus2.recount-1];//计算读取的CRC校验位
	//上述2行等价于下面这条语句//rccrc=modbus2.rcbuf[modbus2.recount-1]|(((uint16_t)modbus2.rcbuf[modbus2.recount-2])<<8);//获取接收到的CRC
	if(crc == rccrc) //CRC检验成功，开始分析数据包
	{	
	   if(modbus2.rcbuf[0] == modbus2.myadd)//【1】该数据包首ID位是自己的从机地址
		 {
		   switch(modbus2.rcbuf[1])//根据功能码相应地做不同处理【0x03-0x06-0x16三种功能码】
			 {
				 case 0:break;
				 case 1:break;
				 case 2:break;
				 
				 case 3:modbus2_Func3();break;//主机读取从机寄存器中的数据
				 
				 case 4:break;
				 case 5:break;
				 
         case 6:modbus2_Func6();break;//主机向从机的1个寄存器写入值
				 
				 case 7:break;
				 case 8:break;
				 
				 case 9:break;
				 
				 case 16:modbus2_Func16();break;//主机向从机的多个寄存器写入数据
			 }
		 }
		else if(modbus2.rcbuf[0] == 0){}//【2】ID位为0，则为广播地址不予回应	  
	}	

	// 尽早释放接收窗口，避免在后续阻塞操作期间丢失接收中断链路
	modbus2.recount = 0;
	modbus2.reflag = 0;
		
	//通过串口3、6向手腕电机发送指令
	if(Reg[14]%256 == 0x01){
	vel_move(Reg[0]/256,Reg[0]%256,Reg[1]/256,Reg[1]%256);//位置(0~2000步)+速度(0~4000步/s),高位在前、低位在后
	vel_move1(Reg[2]/256,Reg[2]%256,Reg[3]/256,Reg[3]%256);}
	wrist_enabled = Reg[14]/256;
	cycle_time = Reg[15];
//	set_motion_period(Reg[15]);
//	set_motion_position(Reg[16],Reg[17]);
	
//	HAL_Delay(50);
	//通过串口1-1号485总线向手指从驱动板发送指令
	for(i=0;i<5;i++){Palm1[i] = Reg[i+4];Palm2[i] = Reg[i+9];}
	Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, Palm1);
	HAL_Delay(100);
	Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, Palm2);
	
	modbus2.recount = 0;//接收计数清零
	modbus2.reflag = 0; //接收标志位清零
}     

/*
********************************************************************************
主机：03——功能码0x03，主机读取从机寄存器中的数据
01  03      00 01     00 01          D5 CA	从起始地址0001开始读读取1个（0001个）寄存器的数据内容
ID 功能码  起始地址  读取的寄存器个数  CRC校验
从机返回：
01  03       02       00 03          F8 45  返回了1个寄存器中2个字节的数据，该数据是00 03
ID 功能码 返回的字节数 返回的数据内容  CRC校验
********************************************************************************
*/
// modbus2 3号功能码函数——主机读取从机寄存器中的数据
void modbus2_Func3()
{
	//【1】从机接收主机数据包，拆包处理
	uint16_t Regadd;//16位起始地址
	uint16_t Reglen;//16位要读取的寄存器个数
	uint16_t crc;//16位校验码
	uint16_t i,j;	
	Regadd = modbus2.rcbuf[2]*256+modbus2.rcbuf[3];//要读取的寄存器起始地址（由rcbuf[2][3]高八位+低八位表示）
	Reglen = modbus2.rcbuf[4]*256+modbus2.rcbuf[5];//要读取的寄存器个数（由rcbuf[4][5]高八位+低八位表示）
	//【2】从机返回：将返回数据打包
	i = 0;
	modbus2.sendbuf[i++] = modbus2.myadd;      //sendbuf[0]ID号，即从机地址，执行后i+1
	modbus2.sendbuf[i++] = 0x03;              //sendbuf[1]功能码，执行后i+1
	modbus2.sendbuf[i++] = ((Reglen*2)%256);  //sendbuf[2]返回的字节个数，执行后i+1
	for(j=0;j<Reglen;j++)//根据要读取的寄存器个数Reglen，逐个赋值数据内容
	{
		//reg是提前定义好的16位数组（模仿寄存器）
		modbus2.sendbuf[i++] = Reg[Regadd+j]/256;//sendbuf[3]高位数据，执行后i+1
		modbus2.sendbuf[i++] = Reg[Regadd+j]%256;//sendbuf[4]低位数据，执行后i+1
	}
	crc = Modbus_CRC16(modbus2.sendbuf,i);    //计算要返回数据的CRC
	modbus2.sendbuf[i++] = crc/256;//sendbuf[5]校验位高位，执行后i+1
	modbus2.sendbuf[i++] = crc%256;//sendbuf[6]校验位低位，执行后i+1

	//【3】从机返回：将数据包发送给主机
//	RS485_TX_ENABLE;//使能485控制端(启动发送)
	for(j=0;j<i;j++){modbus2_Send_Byte(modbus2.sendbuf[j]);}//发送数据
//	HAL_UART_Transmit(&huart2, modbus2.sendbuf, sizeof(modbus2.sendbuf), 100);//发送数据
//    while(!(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC))); // 等待发送完成
//	RS485_RX_ENABLE;//失能485控制端（改为接收）
}
/*
********************************************************************************
主机：06——功能码0x06，主机向从机的1个寄存器写入值
01  06      00 04     00 05          08 08    从起始地址0004的1个寄存器写入数据0005
ID 功能码  起始地址  写入的数据内容    CRC校验
从机返回：
01  06      00 04     00 05          08 08    从机返回与主机完全一致
ID 功能码  起始地址  写入的数据内容    CRC校验
********************************************************************************
*/
// modbus2 6号功能码函数——主机向从机的1个寄存器写入数据
void modbus2_Func6()  
{
	//【1】从机接收主机数据包，拆包处理
	uint16_t Regadd;//16位起始地址
	uint16_t val;//16位要写入的数据内容
	uint16_t crc;//16位校验码
	uint16_t i,j;		
	Regadd=modbus2.rcbuf[2]*256+modbus2.rcbuf[3];  //要写入的寄存器起始地址（由rcbuf[2][3]高八位+低八位表示） 
	val=modbus2.rcbuf[4]*256+modbus2.rcbuf[5];     //要写入的数据内容（由rcbuf[4][5]高八位+低八位表示）
	Reg[Regadd]=val;  //根据上述Regadd和val修改相应的从机寄存器数值	
	//【2】从机返回：将返回数据打包
	i=0;
	modbus2.sendbuf[i++]=modbus2.myadd;//sendbuf[0]ID号，即从机地址
	modbus2.sendbuf[i++]=0x06;        //sendbuf[1]功能码 
	modbus2.sendbuf[i++]=Regadd/256;  //sendbuf[2][3]起始地址高八位+低八位
	modbus2.sendbuf[i++]=Regadd%256;
	modbus2.sendbuf[i++]=val/256;     //sendbuf[4][5]写入的数据内容高八位+低八位
	modbus2.sendbuf[i++]=val%256;
	crc=Modbus_CRC16(modbus2.sendbuf,i);//计算crc校验位
	modbus2.sendbuf[i++]=crc/256;     //sendbuf[6][7]crc校验位高八位+低八位
	modbus2.sendbuf[i++]=crc%256;
	//【3】从机返回：将数据包发送给主机
//	RS485_TX_ENABLE;;//使能485控制端(启动发送)  
	for(j=0;j<i;j++){modbus2_Send_Byte(modbus2.sendbuf[j]);}//发送数据
//	HAL_UART_Transmit(&huart2, modbus2.sendbuf, sizeof(modbus2.sendbuf), 100);//发送数据
//    while(!(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC))); // 等待发送完成
//	RS485_RX_ENABLE;//失能485控制端（改为接收）
}
/*
********************************************************************************
主机：16——功能码0x10，主机向从机的多个寄存器写入数据
01  10      00 05      00 02              04        01 02 03 04       92 9F	起始地址0005开始向寄存器1写入0102，向寄存器2写入0304
ID 功能码  起始地址  写入的寄存器个数  写入的字节数   写入的数据内容     CRC校验
从机返回：
01  10      00 05      00 02             51 C9      主机发来的数据包前6个字节+2个字节CRC校验
ID 功能码  起始地址  写入的寄存器个数     CRC校验
********************************************************************************
*/



// modbus2 16号功能码函数——主机向从机的多个寄存器写入值
void modbus2_Func16()
{
	//【1】从机接收主机数据包，拆包处理
	uint16_t Regadd;//16位起始地址
	uint16_t Reglen;//16位要写入的寄存器个数
	uint16_t crc;//16位校验码
	uint16_t i,j;
	Regadd=modbus2.rcbuf[2]*256+modbus2.rcbuf[3];  //要写入的寄存器起始地址（由rcbuf[2][3]高八位+低八位表示）
	Reglen = modbus2.rcbuf[4]*256+modbus2.rcbuf[5];//要写入的寄存器个数（由rcbuf[4][5]高八位+低八位表示）
	for(i=0;i<Reglen;i++)//根据上述Regadd和val修改相应的从机寄存器数值
	{
		//rcbuf[6]是要写入的字节个数==Reglen*2，接收数组的第七位开始是数据
		Reg[Regadd+i]=modbus2.rcbuf[7+i*2]*256+modbus2.rcbuf[8+i*2];//对寄存器1写入rcbuf[7][8]高八位+低八位，然后进入下一个循环
	}	
	//【2】从机返回：将返回数据打包，数据包内容==接收数组rcbuf的前6字节+2个字节CRC校验
	modbus2.sendbuf[0]=modbus2.rcbuf[0];//sendbuf[0]ID号，即从机地址
	modbus2.sendbuf[1]=modbus2.rcbuf[1];//sendbuf[1]功能码 
	modbus2.sendbuf[2]=modbus2.rcbuf[2];//sendbuf[2][3]起始地址高八位+低八位
	modbus2.sendbuf[3]=modbus2.rcbuf[3];
	modbus2.sendbuf[4]=modbus2.rcbuf[4];//sendbuf[4][5]写入的寄存器个数高八位+低八位
	modbus2.sendbuf[5]=modbus2.rcbuf[5];
	crc=Modbus_CRC16(modbus2.sendbuf,6);//计算crc校验位
	modbus2.sendbuf[6]=crc/256;        //sendbuf[6][7]crc校验位高八位+低八位
	modbus2.sendbuf[7]=crc%256;
	//【3】从机返回：将数据包发送给主机	
//	RS485_TX_ENABLE;;//使能485控制端(启动发送)  
	for(j=0;j<8;j++){modbus2_Send_Byte(modbus2.sendbuf[j]);}//发送数据
	
}



//--------------------------------------------------串口1对应modbus1主机部分程序，与手指485相连----------------------------------------------------
//0x06功能：主机向从机的一个寄存器中写入指定数据
void Host_write06_slave(uint8_t slave,uint8_t fun,uint16_t StartAddr,uint16_t num)//从机ID-功能码-写入的起始地址-写入的数据内容
{
	//【1】根据函数各个参数打包数据包
	uint16_t crc,j;//计算的CRC校验位
	modbus1.slave_add=slave;//从机地址赋值一下，后期有用
	modbus1.Host_Txbuf[0]=slave;//这是要匹配的从机地址
	modbus1.Host_Txbuf[1]=fun;//功能码
	modbus1.Host_Txbuf[2]=StartAddr/256;//起始地址高位
	modbus1.Host_Txbuf[3]=StartAddr%256;//起始地址低位
	modbus1.Host_Txbuf[4]=num/256;
	modbus1.Host_Txbuf[5]=num%256;
	crc=Modbus_CRC16(&modbus1.Host_Txbuf[0],6); //获取CRC校验位
	modbus1.Host_Txbuf[6]=crc/256;//寄存器个数高位
	modbus1.Host_Txbuf[7]=crc%256;//寄存器个数低位
	   
	//【2】将上述打包好的数据包发送给指定从机
	for(j=0;j<8;j++)
	{
		modbus1_Send_Byte(modbus1.Host_Txbuf[j]);
	}
	HAL_Delay(10);
	modbus1.Host_send_flag=1;//表示发送数据完毕
}
//主机接收到从机的返回数据
void Host_Func6()
{
	uint16_t crc,rccrc;
	crc = Modbus_CRC16(&modbus1.rcbuf[0],6); //获取CRC校验位
	rccrc = modbus1.rcbuf[6]*256+modbus1.rcbuf[7];//计算读取的CRC校验位
	if(crc == rccrc) //CRC检验成功 开始分析包
	{	
	   if(modbus1.rcbuf[0] == modbus1.slave_add)  // 检查地址是是对应从机发过来的
		 {
			 if(modbus1.rcbuf[1]==6)//功能码是06
			 {		
//				printf("向地址为 %d 的从机寄存器 %d 中写入数据 %d \r\n ",
//					(int)modbus1.rcbuf[0] , (int)modbus1.rcbuf[3]+((int)modbus1.rcbuf[2])*256 , (int)modbus1.rcbuf[5]+((int)modbus1.rcbuf[4])*256);
//				printf("0x06功能已实现!\r\n");				
			 }
		 }		 
	}			
		modbus1.Host_End=1;//接收的数据处理完毕
}
//0x16功能：主机向从机的多个寄存器中写入指定数据
void Host_write16_slave(uint8_t slave, uint8_t fun, uint16_t StartAddr, uint16_t REnum, uint8_t BYTEnum, uint16_t* ModbusData)
						//从机ID-功能码-写入的起始地址-写入的寄存器个数-待写入的字节数——写入的数据内容1,2,3,4,5，针对手指驱动板4电机+1模式设计的
{
	//【1】根据函数各个参数打包数据包
	uint16_t crc,j;//计算的CRC校验位
	modbus1.slave_add=slave;//从机地址赋值一下，后期有用
	modbus1.Host_Txbuf[0]=slave;//这是要匹配的从机地址
	modbus1.Host_Txbuf[1]=fun;//功能码
	modbus1.Host_Txbuf[2]=StartAddr/256;//起始地址高位
	modbus1.Host_Txbuf[3]=StartAddr%256;//起始地址低位
	modbus1.Host_Txbuf[4]=REnum/256;//寄存器个数高位
	modbus1.Host_Txbuf[5]=REnum%256;//寄存器个数低位
	modbus1.Host_Txbuf[6]=BYTEnum;//写入的字节个数 = 寄存器个数*2

	modbus1.Host_Txbuf[7]=ModbusData[0]/256;//[0]数据1-高位
	modbus1.Host_Txbuf[8]=ModbusData[0]%256;//[0]数据1-低位
	modbus1.Host_Txbuf[9]=ModbusData[1]/256;//[1]数据2-高位
	modbus1.Host_Txbuf[10]=ModbusData[1]%256;//[1]数据2-低位	
	modbus1.Host_Txbuf[11]=ModbusData[2]/256;//[2]数据3-高位
	modbus1.Host_Txbuf[12]=ModbusData[2]%256;//[2]数据3-低位
	modbus1.Host_Txbuf[13]=ModbusData[3]/256;//[3]数据4-高位
	modbus1.Host_Txbuf[14]=ModbusData[3]%256;//[3]数据4-低位
	modbus1.Host_Txbuf[15]=ModbusData[4]/256;//[4]数据5-高位
	modbus1.Host_Txbuf[16]=ModbusData[4]%256;//[4]数据5-低位
	
	crc=Modbus_CRC16(&modbus1.Host_Txbuf[0],17); //获取CRC校验位
	modbus1.Host_Txbuf[17]=crc/256;//寄存器个数高位
	modbus1.Host_Txbuf[18]=crc%256;//寄存器个数低位
	   
	//【2】将上述打包好的数据包发送给指定从机
	for(j=0;j<19;j++)
	{
		modbus1_Send_Byte(modbus1.Host_Txbuf[j]);
	}
	HAL_Delay(10);
	modbus1.Host_send_flag=1;//表示发送数据完毕
}
//主机接收到从机的返回数据
void Host_Func16(void)
{
	uint16_t crc,rccrc;
	crc = Modbus_CRC16(&modbus1.rcbuf[0],6); //获取CRC校验位
	rccrc = modbus1.rcbuf[6]*256+modbus1.rcbuf[7];//计算读取的CRC校验位
	if(crc == rccrc) //CRC检验成功 开始分析包
	{	
	   if(modbus1.rcbuf[0] == modbus1.slave_add)  // 检查地址是是对应从机发过来的
		 {
			 if(modbus1.rcbuf[1]==16)//功能码是16
			 {		
//				printf("\r\n向从机 %d 的起始地址 %d 中写入 %d 个寄存器 ",
//					(int)modbus1.rcbuf[0] , (int)modbus1.rcbuf[3]+((int)modbus1.rcbuf[2])*256 , (int)modbus1.rcbuf[5]+((int)modbus1.rcbuf[4])*256);
//				printf("\r\n0x16功能已实现!");				
			 }
		 }		 
	}			
		modbus1.Host_End=1;//接收的数据处理完毕
}

////CRC16校验计算函数，数据包中除CRC的2个校验位之外的所有数据均参与计算
//uint16_t Modbus_CRC16(uint8_t *pData, uint16_t len)
//{
//    uint16_t crc = 0xFFFF;
//    for(int pos = 0; pos < len; pos++) {
//        crc ^= (uint16_t)pData[pos];
//        for(int i = 8; i != 0; i--) {
//            if((crc & 0x0001) != 0) {
//                crc >>= 1;
//                crc ^= 0xA001;
//            } else {
//                crc >>= 1;
//            }
//        }
//    }
//    return crc;
//}
void Continuous_Motion(uint8_t flag,uint8_t count,uint8_t mode)
{



}


