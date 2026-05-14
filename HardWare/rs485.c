#include "rs485.h"
#include "usart.h"

MODBUS modbus;//定义MODBUS结构体类型的变量modbus
uint16_t Reg[100] = {0};//从机存储的数据，均是16位的数据

//485串口发一个字节数据的函数
void Modbus_Send_Byte(uint8_t ch)
{
	/* 发送一个字节数据到USART1 */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xff);	
}



// Modbus初始化函数，给定本机作为从机时的地址
void Modbus_Init()
{
	//为从机寄存器赋值，全局变量赋值必须在函数内进行！！！
	Reg[0] = 0x0000;//motor1出轴角度【高八位】+速度【低八位】
	Reg[1] = 0x0000;//motor2出轴角度【高八位】+速度【低八位】
	Reg[2] = 0x0000;//motor3出轴角度【高八位】+速度【低八位】
	Reg[3] = 0x0000;//motor4出轴角度【高八位】+速度【低八位】
	Reg[4] = 0x0000;//运行模式位【高八位】+启停标志位【低八位】
	modbus.myadd = 0x02; //从机设备地址为1
	modbus.timrun = 0;    //modbus定时器停止计算
//	modbus.slave_add=0x01;//主机要匹配的从机地址（本设备作为主机时）
}

// Modbus事件处理函数
void Modbus_Event()
{
	uint16_t crc,rccrc;//crc和接收到的crc
	//【1】没有收到数据包，return退出函数
	if(modbus.reflag == 0){return;}//如果接收未完成则返回空，不做处理
	//【2】收到数据包(已经接收完成)：
	//先根据读到的数据帧做CRC校验：参数1是数组首地址，参数2是要计算的长度（除了CRC校验位其余全算）
	crc = Modbus_CRC16(&modbus.rcbuf[0],modbus.recount-2); //获取CRC校验位
	rccrc = modbus.rcbuf[modbus.recount-2]*256+modbus.rcbuf[modbus.recount-1];//计算读取的CRC校验位
	//上述2行等价于下面这条语句//rccrc=modbus.rcbuf[modbus.recount-1]|(((uint16_t)modbus.rcbuf[modbus.recount-2])<<8);//获取接收到的CRC
	if(crc == rccrc) //CRC检验成功，开始分析数据包
	{	
	   if(modbus.rcbuf[0] == modbus.myadd)//【1】该数据包首ID位是自己的从机地址
		 {
		   switch(modbus.rcbuf[1])//根据功能码相应地做不同处理【0x03-0x06-0x16三种功能码】
			 {
				 case 0:break;
				 case 1:break;
				 case 2:break;
				 case 3:Modbus_Func3();break;//主机读取从机寄存器中的数据
				 case 4:break;
				 case 5:break;
                 case 6:Modbus_Func6();break;//主机向从机的1个寄存器写入值
				 case 7:break;
				 case 8:break;
				 case 9:break;
				 case 16:Modbus_Func16();break;//主机向从机的多个寄存器写入数据
			 }
		 }
		else if(modbus.rcbuf[0] == 0){}//【2】ID位为0，则为广播地址不予回应	  
	}	
	modbus.recount = 0;//接收计数清零
	modbus.reflag = 0; //接收标志位清零
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
// Modbus 3号功能码函数——主机读取从机寄存器中的数据
void Modbus_Func3()
{
	//【1】从机接收主机数据包，拆包处理
	uint16_t Regadd;//16位起始地址
	uint16_t Reglen;//16位要读取的寄存器个数
	uint16_t crc;//16位校验码
	uint16_t i,j;	
	Regadd = modbus.rcbuf[2]*256+modbus.rcbuf[3];//要读取的寄存器起始地址（由rcbuf[2][3]高八位+低八位表示）
	Reglen = modbus.rcbuf[4]*256+modbus.rcbuf[5];//要读取的寄存器个数（由rcbuf[4][5]高八位+低八位表示）
	//【2】从机返回：将返回数据打包
	i = 0;
	modbus.sendbuf[i++] = modbus.myadd;      //sendbuf[0]ID号，即从机地址，执行后i+1
	modbus.sendbuf[i++] = 0x03;              //sendbuf[1]功能码，执行后i+1
	modbus.sendbuf[i++] = ((Reglen*2)%256);  //sendbuf[2]返回的字节个数，执行后i+1
	for(j=0;j<Reglen;j++)//根据要读取的寄存器个数Reglen，逐个赋值数据内容
	{
		//reg是提前定义好的16位数组（模仿寄存器）
		modbus.sendbuf[i++] = Reg[Regadd+j]/256;//sendbuf[3]高位数据，执行后i+1
		modbus.sendbuf[i++] = Reg[Regadd+j]%256;//sendbuf[4]低位数据，执行后i+1
	}
	crc = Modbus_CRC16(modbus.sendbuf,i);    //计算要返回数据的CRC
	modbus.sendbuf[i++] = crc/256;//sendbuf[5]校验位高位，执行后i+1
	modbus.sendbuf[i++] = crc%256;//sendbuf[6]校验位低位，执行后i+1

	//【3】从机返回：将数据包发送给主机
//	RS485_TX_ENABLE;//使能485控制端(启动发送)
	for(j=0;j<i;j++){Modbus_Send_Byte(modbus.sendbuf[j]);}//发送数据
//	HAL_UART_Transmit(&huart1, modbus.sendbuf, sizeof(modbus.sendbuf), 100);//发送数据
//    while(!(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC))); // 等待发送完成
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
// Modbus 6号功能码函数——主机向从机的1个寄存器写入数据
void Modbus_Func6()  
{
	//【1】从机接收主机数据包，拆包处理
	uint16_t Regadd;//16位起始地址
	uint16_t val;//16位要写入的数据内容
	uint16_t crc;//16位校验码
	uint16_t i;
//	uint16_t j;	
	Regadd=modbus.rcbuf[2]*256+modbus.rcbuf[3];  //要写入的寄存器起始地址（由rcbuf[2][3]高八位+低八位表示） 
	val=modbus.rcbuf[4]*256+modbus.rcbuf[5];     //要写入的数据内容（由rcbuf[4][5]高八位+低八位表示）
	Reg[Regadd]=val;  //根据上述Regadd和val修改相应的从机寄存器数值	
	//【2】从机返回：将返回数据打包
	i=0;
	modbus.sendbuf[i++]=modbus.myadd;//sendbuf[0]ID号，即从机地址
	modbus.sendbuf[i++]=0x06;        //sendbuf[1]功能码 
	modbus.sendbuf[i++]=Regadd/256;  //sendbuf[2][3]起始地址高八位+低八位
	modbus.sendbuf[i++]=Regadd%256;
	modbus.sendbuf[i++]=val/256;     //sendbuf[4][5]写入的数据内容高八位+低八位
	modbus.sendbuf[i++]=val%256;
	crc=Modbus_CRC16(modbus.sendbuf,i);//计算crc校验位
	modbus.sendbuf[i++]=crc/256;     //sendbuf[6][7]crc校验位高八位+低八位
	modbus.sendbuf[i++]=crc%256;
	
//	//【3】从机返回：将数据包发送给主机
////	RS485_TX_ENABLE;;//使能485控制端(启动发送)  
//	for(j=0;j<i;j++){Modbus_Send_Byte(modbus.sendbuf[j]);}//发送数据
////	HAL_UART_Transmit(&huart1, modbus.sendbuf, sizeof(modbus.sendbuf), 100);//发送数据
////    while(!(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC))); // 等待发送完成
////	RS485_RX_ENABLE;//失能485控制端（改为接收）
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
// Modbus 16号功能码函数——主机向从机的多个寄存器写入值
void Modbus_Func16()
{
	//【1】从机接收主机数据包，拆包处理
	uint16_t Regadd;//16位起始地址
	uint16_t Reglen;//16位要写入的寄存器个数
	uint16_t crc;//16位校验码
	uint16_t i;
//	uint16_t j;	
	Regadd=modbus.rcbuf[2]*256+modbus.rcbuf[3];  //要写入的寄存器起始地址（由rcbuf[2][3]高八位+低八位表示）
	Reglen = modbus.rcbuf[4]*256+modbus.rcbuf[5];//要写入的寄存器个数（由rcbuf[4][5]高八位+低八位表示）
	for(i=0;i<Reglen;i++)//根据上述Regadd和val修改相应的从机寄存器数值
	{
		//rcbuf[6]是要写入的字节个数==Reglen*2，接收数组的第七位开始是数据
		Reg[Regadd+i]=modbus.rcbuf[7+i*2]*256+modbus.rcbuf[8+i*2];//对寄存器1写入rcbuf[7][8]高八位+低八位，然后进入下一个循环
	}	
	//【2】从机返回：将返回数据打包，数据包内容==接收数组rcbuf的前6字节+2个字节CRC校验
	modbus.sendbuf[0]=modbus.rcbuf[0];//sendbuf[0]ID号，即从机地址
	modbus.sendbuf[1]=modbus.rcbuf[1];//sendbuf[1]功能码 
	modbus.sendbuf[2]=modbus.rcbuf[2];//sendbuf[2][3]起始地址高八位+低八位
	modbus.sendbuf[3]=modbus.rcbuf[3];
	modbus.sendbuf[4]=modbus.rcbuf[4];//sendbuf[4][5]写入的寄存器个数高八位+低八位
	modbus.sendbuf[5]=modbus.rcbuf[5];
	crc=Modbus_CRC16(modbus.sendbuf,6);//计算crc校验位
	modbus.sendbuf[6]=crc/256;        //sendbuf[6][7]crc校验位高八位+低八位
	modbus.sendbuf[7]=crc%256;
	
//	//【3】从机返回：将数据包发送给主机	
////	RS485_TX_ENABLE;;//使能485控制端(启动发送)  
//	for(j=0;j<8;j++){Modbus_Send_Byte(modbus.sendbuf[j]);}//发送数据
////	HAL_UART_Transmit(&huart1, modbus.sendbuf, sizeof(modbus.sendbuf), 100);//发送数据
////    while(!(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC))); // 等待发送完成
////	RS485_RX_ENABLE;//失能485控制端（改为接收）
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



