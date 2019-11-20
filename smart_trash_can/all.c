/*
连线说明:
步进电机28BYJ48 IN1~IN4 接 P2^0~P2^3;
红外避障模块(开盖) 接 P2^7;
红外避障模块(满溢) 接 P2^6;
温度DS18B20 DQ 接 P3^7;
烟雾检测MQ-7 DOUT 接P2^5;
蜂鸣器 接 P2^4;
*/

#include <reg52.h>
#include<intrins.h>
#define uchar unsigned char
#define uint unsigned int
 
uchar code CCW[8]={0x08,0x0c,0x04,0x06,0x02,0x03,0x01,0x09};//逆时钟旋转相序表
uchar code CW[8]={0x09,0x01,0x03,0x02,0x06,0x04,0x0c,0x08};//正时钟旋转相序表
//数码管
uchar table[]={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};
//带小数点
uchar code table1[]={0x40,0x79,0x24,0x30,0x19,0x12,0x02,0x78,0x00,0x10};
uchar code T_COM[] = {0xfe, 0xfd,0xfb,0xf7};//数码管位码
uchar disbuff[4]={0,0,0,0};
uint temp=0;//温度值 
sbit OUT0=P2^7;//距离感应人是否靠近
sbit OUT1=P2^6;//满溢检测，当满了后蜂鸣器响起
sbit FMQ=P2^4; //蜂鸣器
sbit K1=P3^3;  //复位按键
sbit DOUT=P2^5;  //MQ-7
sbit DS =P3^7;	  //DS18B20接P3^6口
uint flag,j=0;

void delayMS(uint a){//延时a毫秒
  uchar b;
  while(a--){
   for(b=0;b<115;b++);
  }
}
void delayS(uint a){//延时a秒
    uchar b,c,d;
	for(;a>0;a--){
		for(c=167;c>0;c--)
       		for(b=171;b>0;b--)
            	for(d=16;d>0;d--);
	}
}
/*------------------------蜂鸣器模块------------------------*/
//当温度过高、满溢和有毒气体时就会响起，K4(JD51)用于复位
void delay500us(void){//延时500us
  int j;
  for(j=0;j<57;j++);
}
void beep(void){//蜂鸣器
  uchar t;
  flag=1;
  while(flag){
	  for(t=0;t<100;t++){
	   	delay500us();
	 	FMQ=!FMQ;
	  }
	  if(K1==0){
		flag=0;
	  }
	  FMQ=1;
  }
}
/*------------------------步进电机模块------------------------*/
void motor_ccw(void){//逆时针旋转
  uchar i,j;
  for(j=0;j<8;j++){//电机内部旋转一周
	  for(i=0;i<8;i++){//旋转45度
	  	P1=CCW[i];
		delayMS(2);//调节转速
	  }
  }
}

void motor_cw(void){//顺时针旋转
  uchar i,j;
  for(j=0;j<8;j++){
	  for(i=0;i<8;i++){
	  	P1=CW[i];
		delayMS(2);
	  }
  }
}

/*------------------------温度------------------------*/
void Delay_us(uchar us)
{
	while(us--);	
}

/*单总线初始化时序*/
bit ds_init()
{
	bit i;
	DS = 1;
	_nop_();
	DS = 0;
	Delay_us(75); //拉低总线499.45us 挂接在总线上的18B20将会全部被复位
	DS = 1; //释放总线
	Delay_us(4); //延时37.95us 等待18B20发回存在信号
	i = DS;
	Delay_us(20); //141.95us
	DS = 1;
	_nop_();
	return (i);
}
/*写一个字节*/
void write_byte(uchar dat)
{
	uchar i;
	for(i=0;i<8;i++)
	{
		DS = 0;
		_nop_();//产生些时序
		DS = dat & 0x01;
		Delay_us(10);//76.95us
		DS = 1; //释放总线准备下一次数据写入
		_nop_();
		dat >>= 1;
	}
}
uchar read_byte()
{
	uchar i, j, dat;
	for(i=0;i<8;i++)
	{
		DS = 0;
		_nop_();//产生读时序
		DS = 1;
		_nop_();//释放总线
		j = DS;
		Delay_us(10);//76.95us
		DS = 1;
		_nop_();
		dat = (j<<7)|(dat>>1);	
	}
	return (dat);
}

void convert(void){ 
	disbuff[0]=temp/1000;      		 //十位
	disbuff[1]=temp%1000/100;      	 //个位
	disbuff[2]=temp%100/10;      	//小数点后一位
	disbuff[3]=temp%10;      		 //小数点后两位
}
void Display(void)//扫描数码管
{
	P0=0xFF;
	P2=T_COM[j];
  if(j!=1){
		P0=table[disbuff[j]];
	}else{
		P0=table1[disbuff[j]];
	}
	j++;
	if(j==4)
		j=0;
}
void  zd3() interrupt 3//T1中断用来扫描数码管
{
	TH1 = 0x0FE;
	TL1 = 0x0C;
	Display(); 
}
void  zd1() interrupt 1//T0中断用来测温度
{
	uint i;
	uchar L, M;
	TH0 = 0x3C;//测温度
	TL0 = 0x0B0;
	ds_init();//初始化DS18B20
	write_byte(0xcc);//发送跳跃ROM指令
	write_byte(0x44);//发送温度转换指令
	ds_init();//初始化DS18B20
	write_byte(0xcc);//发送跳跃ROM指令
	write_byte(0xbe);//读取DS18B20暂存器值
	L = read_byte();
	M = read_byte();
	i = M;
	i <<= 8;
	i |= L;
	//temp = i * 0.0625 * 10 + 0.5;
	temp = i *6.25;//temp为实际温度的100倍
	convert();
}
/*------------------------主函数------------------------*/
void main(void){
	uchar r;
	uchar N=16;//因为步进电机是减速步进电机，减速比的1/64，64为旋转360，16为旋转90度
	TMOD=0x11;
	TH0 = 0x3C;//测温度
  TL0 = 0x0B0;
	         
  TH1 = 0x0FE;
  TL1 = 0x0C;
	ET0=1;			   //允许T0中断
	TR0=1;			   //开启定时器
	ET1=1;			   //允许T1中断
	TR1=1;			   //开启定时器

	EA=1;			   //开启总中断
	
	//延时1秒，温度采集完成
	delayS(1);
	
	while(1){
	 	if(OUT0==0){
			//有人靠近
		   	for(r=0;r<N;r++){
		    	motor_ccw();  //电机逆转90度
			}
			while(!OUT0);//直到人离开
			delayS(5);
			for(r=0;r<N;r++){
			    motor_cw();  //电机正转90度
			}
			if(OUT1==0){
				//垃圾桶满
				delay500us();//延时抗干扰
				if(OUT1==0){
					 beep();
				}
			}
		}else{
			P1=0xf0;    //电机停止
		}
		if(DOUT==0||temp>3500)//有害气体浓度过高或温度过高
		{
		 	delay500us();//延时抗干扰
			if(DOUT==0||temp>350){
			  	beep();
			}
		}
	 }
}          