#include "Arm_Move_bsp.h"
#include "math.h"
Arm_Ctrl Arm_Ctrl_Info = {
	.Servo_0 = 90.0f,.Servo_1 = -47.0f,.Servo_2 = 90,.Servo_3 = -3,.Servo_4 = 110.0f,.Status = Arm_Ctrl_Debug
	,.Speed_0 = 800,.Speed_1 = 800,.Speed_2 = 800,.Speed_3 = 800,.Speed_4 = 800,.Speed_all = 80,
};

float theta_0[4]={90,-47,90,-3};
Arm_Order Arm_Order_Info;
Arm_Locus Arm_Locus_Info;
Arm_Theta Arm_Theta_Info;

Arm_Locus Arm_Locus_Info ={
  .x_0 =172.0f,.y_0 =-155.0f,.z_0 =76.0f,.x_1 =174.f,.y_1 =-7.5f
	,.z_1 =76.0f,.x_2 =167.0f,.y_2=154.5f,.z_2=72.0f,
};

/*逆向运动学*/
void Inverse_Kinematic(float x_0, float y_0, float z_0){
	float theta0;
	float theta1;
	float theta2;
	float theta3;
	float Pi=3.14159;
	float L1=84.7;
  float L2=81;
  float L3=102.5;
  float L4=122;
 //求角度0//
	if(y_0<=0 && x_0<=0){
		y_0=-y_0;
		x_0=-x_0;
		theta0 = atan2(y_0,x_0);
		theta0 = Pi+theta0;
		theta0 = -theta0;
	}
	else{
		theta0 = atan2(y_0,x_0);
	  theta0 = -theta0;
	}
//求角度2//
	float r;
	r = sqrt(pow(x_0,2)+pow(y_0,2));     //引入实际运动平面
	theta2 = acos((pow(r-L4,2)+pow(z_0-L1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3));
//求角度1//
	float vecsin;
	float veccos;         //几何关系推导，矩阵的逆
	vecsin = (r-L4)*(L2+L3*cos(theta2))-L3*sin(theta2)*(z_0-L1);
	veccos = (r-L4)*L3*sin(theta2)+(z_0-L1)*(L2+L3*cos(theta2));
	theta1 = atan2(vecsin,veccos);
//角度3//
	theta3 = Pi/2-theta1-theta2;
//指令角度//
	Arm_Theta_Info.theta_0=theta0/Pi*180+theta_0[0];
	Arm_Theta_Info.theta_1=theta1/Pi*180+theta_0[1];
	Arm_Theta_Info.theta_2=-theta2/Pi*180+theta_0[2];
	Arm_Theta_Info.theta_3=theta3/Pi*180+theta_0[3];
}

/*机械臂响应速度函数*/
void Arm_Invesal(uint16_t Speed){
  Arm_Ctrl_Info.Speed_0 = Speed;
	Arm_Ctrl_Info.Speed_1 = Speed;
	Arm_Ctrl_Info.Speed_2 = Speed;
	Arm_Ctrl_Info.Speed_3 = Speed;
	Arm_Ctrl_Info.Speed_4 = Speed;
}

/*机械臂爪子翻转函数*/
void Arm_Move_Reversal(void){
  if(Arm_Theta_Info.theta_4 == 45){
	  Arm_Theta_Info.theta_4 = 85;
	}
	else{
	  Arm_Theta_Info.theta_4 = 45;
	}
}

/*机械臂运动到达函数*/
void Arm_Move_Front(void){
	Inverse_Kinematic(Arm_Locus_Info.x,Arm_Locus_Info.y,Arm_Locus_Info.z);
  Arm_Ctrl_Info.Servo_0 = Arm_Theta_Info.theta_0;
	osDelay(300);//400
	Arm_Ctrl_Info.Servo_1 = Arm_Theta_Info.theta_1;
	Arm_Ctrl_Info.Servo_2 = Arm_Theta_Info.theta_2;
	Arm_Ctrl_Info.Servo_3 = Arm_Theta_Info.theta_3;
	osDelay(200);//400
}

/*机械臂过渡态函数*/
void Arm_Move_Transition_F(float x_tf,float y_tf,float z_tf){
	Inverse_Kinematic(x_tf,y_tf,z_tf);
  Arm_Ctrl_Info.Servo_0 = Arm_Theta_Info.theta_0;
	osDelay(250);//300
	Arm_Ctrl_Info.Servo_2 = Arm_Theta_Info.theta_2;
	Arm_Ctrl_Info.Servo_1 = Arm_Theta_Info.theta_1;
	Arm_Ctrl_Info.Servo_3 = Arm_Theta_Info.theta_3;
	osDelay(100);//100
}
void Arm_Move_Transition_B(float x_tb,float y_tb,float z_tb){
	Inverse_Kinematic(x_tb,y_tb,z_tb);
	Arm_Ctrl_Info.Servo_2 = Arm_Theta_Info.theta_2;
	Arm_Ctrl_Info.Servo_1 = Arm_Theta_Info.theta_1;
	Arm_Ctrl_Info.Servo_3 = Arm_Theta_Info.theta_3;
	osDelay(150);//200
	Arm_Ctrl_Info.Servo_0 = Arm_Theta_Info.theta_0;
	osDelay(100);//100
}

/*机械臂中间态返回函数*/
void Arm_Move_Back(void){
  Arm_Ctrl_Info.Servo_1 = theta_0[1];
	Arm_Ctrl_Info.Servo_2 = theta_0[2];
	Arm_Ctrl_Info.Servo_3 = theta_0[3];
	osDelay(200);//200
	Arm_Ctrl_Info.Servo_0 = theta_0[0];
//	osDelay(100);//100
	Arm_Invesal(800);
}

void Arm_xyz(float x,float y,float z){
  Inverse_Kinematic(x,y,z);
	Arm_Ctrl_Info.Servo_2 = Arm_Theta_Info.theta_2;
	Arm_Ctrl_Info.Servo_1 = Arm_Theta_Info.theta_1;
	Arm_Ctrl_Info.Servo_3 = Arm_Theta_Info.theta_3;
	Arm_Ctrl_Info.Servo_0 = Arm_Theta_Info.theta_0;
}

/*物料盘位置*/
void Arm_Move_Inner(int inner_1,int outer_1){
  if((outer_1 >= 7 && outer_1 <=9) || (outer_1 >=13)){
	  Arm_Theta_Info.theta_4=43;
	}
	else{
	  Arm_Theta_Info.theta_4=80;
	}
	switch (inner_1){
	  case 1:{
			//一号点位位置 -145，85,138
			Arm_Invesal(250);
			Arm_Ctrl_Info.Speed_3 = 200;
			//中间态过渡
      Arm_Move_Transition_F(-150+60,80-30,138+90);
			//目标位置
			Arm_Locus_Info.x=-150;//-150
			Arm_Locus_Info.y=77;//80
			Arm_Locus_Info.z=136;//133;
      Arm_Move_Front();
			osDelay(100);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;//爪子
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//回中间态过渡
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_3 = 300;
      Arm_Move_Transition_B(Arm_Locus_Info.x+60,Arm_Locus_Info.y-30,Arm_Locus_Info.z+70);
			Arm_Ctrl_Info.Speed_3 = 200;
			//回中间态
      Arm_Move_Back();
			break;
		}
		case 2:{
			//二号点位位置-190，-20,142
			Arm_Invesal(250);
			Arm_Ctrl_Info.Speed_0 = 200;
			Arm_Ctrl_Info.Speed_3 = 200;
			//中间态过渡
      Arm_Move_Transition_F(-190+80,-20+8,142+115);
			osDelay(20);
			//目标位置
			Arm_Locus_Info.x=-187;//-187;
			Arm_Locus_Info.y=-30;//-30;
			Arm_Locus_Info.z=138;//138;
      Arm_Move_Front();
			osDelay(100);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//中间态过渡
      Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_3 = 300;
      Arm_Move_Transition_B(Arm_Locus_Info.x+80,Arm_Locus_Info.y+8,Arm_Locus_Info.z+70);
			Arm_Ctrl_Info.Speed_3 = 200;
			//回到中间态
      Arm_Move_Back();
			break;
		}
		case 3:{
			//三号点位位置-128，-110,140
			Arm_Invesal(250);
			Arm_Ctrl_Info.Speed_0 = 200;
			Arm_Ctrl_Info.Speed_3 = 200;
			//中间态过渡
      Arm_Move_Transition_F(-128+50,-112+35,133+115);
			osDelay(60);
			//目标位置
			Arm_Locus_Info.x=-125;//-125;
			Arm_Locus_Info.y=-115;//-115; 
			Arm_Locus_Info.z=133;//133;
      Arm_Move_Front();
			osDelay(100);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//中间态过渡
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_3 = 300;
      Arm_Move_Transition_B(Arm_Locus_Info.x+50,Arm_Locus_Info.y+35,Arm_Locus_Info.z+70);
			Arm_Ctrl_Info.Speed_3 = 200;
			//回到中间态
      Arm_Move_Back();
			break;
		}
	}
}
/*场地物料位置*/
void Arm_Move_Outer(int outer_2){
	if((outer_2 <= 6) || (outer_2 >=10 && outer_2 <=12)){
	  Arm_Theta_Info.theta_4=43;
	}
	else{
	  Arm_Theta_Info.theta_4=80;
	}
  switch (outer_2){
/*原料区 取物料*/
		case 1:{
			//位置1，左上
			Arm_Locus_Info.x=170;//158+6   //158;  //160
			Arm_Locus_Info.y=125;//129+5  //143;  //150
			Arm_Locus_Info.z=203;//203+0.5  //205
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_2=300;
      Arm_Move_Front();
			osDelay(200);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
//	  Arm_Ctrl_Info.Speed_1=200;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			Arm_Invesal(200);
			//中间态过渡

      Arm_Move_Transition_B(Arm_Locus_Info.x-40,Arm_Locus_Info.y-40,Arm_Locus_Info.z+50);
			//回中间态
      Arm_Move_Back();
			
			break;
		}
		case 2:{
			//位置2，中上
			Arm_Locus_Info.x=165;  //165
			Arm_Locus_Info.y=-20;  //-10
			Arm_Locus_Info.z=180;  //185
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_2=400;
      Arm_Move_Front();
			osDelay(200);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
//		Arm_Ctrl_Info.Speed_1=200;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			Arm_Invesal(200);
			//回中间态过渡z上提
      Arm_Move_Transition_B(Arm_Locus_Info.x-40,Arm_Locus_Info.y,Arm_Locus_Info.z+50);
			//回中间态
      Arm_Move_Back();
			break;
		}
		case 3:{
			//位置3，右上
			Arm_Locus_Info.x=163;//   //163;//160;
			Arm_Locus_Info.y=-163; //   //-163;//-150;
			Arm_Locus_Info.z=203;//  //203;//200;
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_2=400;
	    Arm_Move_Front();
			osDelay(200);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
//		Arm_Ctrl_Info.Speed_1=200;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			Arm_Invesal(200);
			//中间态过渡
      Arm_Move_Transition_B(Arm_Locus_Info.x-40,Arm_Locus_Info.y+40,Arm_Locus_Info.z+50);
			//回中间态
      Arm_Move_Back();
			break;
		}
  case 4:{
			Arm_Invesal(100);
			Arm_Ctrl_Info.Servo_4 = 120;
			Arm_Ctrl_Info.Servo_3 = -130;
			Arm_Ctrl_Info.Servo_2 = -40;

			osDelay (250);
			Arm_Invesal(200);
			Arm_Ctrl_Info.Servo_1 = 10;
			osDelay (200);
			
			Arm_Ctrl_Info.Servo_2 = -15;
			Arm_Ctrl_Info.Servo_3 = -110;
			osDelay (200);
			
			Arm_Ctrl_Info.Servo_0 =50;
			osDelay (200);
			Arm_Ctrl_Info.Servo_3 = -90;
			Arm_Ctrl_Info.Servo_1 = 5;
			osDelay (200);
			Arm_Ctrl_Info.Servo_4 = 85;
			Arm_Ctrl_Info.Servo_3 = -70;
			Arm_Ctrl_Info.Servo_2 = 0;

			//目标点
			Arm_Invesal(200);
			Arm_Locus_Info.x=250;//265;
			Arm_Locus_Info.y=150;//150;
			Arm_Locus_Info.z=60;//80;
      Arm_Move_Front();
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(200);
			//准备到过渡点
			Arm_Ctrl_Info.Servo_2 = 60;
			Arm_Ctrl_Info.Servo_3 = -20;
			osDelay (200);
			//过渡点
			Arm_Move_Transition_B(120,120,100);
			osDelay (200);
			//弯折1、2、3号舵机
			Arm_Ctrl_Info.Servo_2 = 0;
			Arm_Ctrl_Info.Servo_1 = -120;
			Arm_Ctrl_Info.Servo_3 = -90;
			osDelay (400);
      Arm_Move_Back();
			osDelay(100);
			break;
		}
		case 5:{
			Arm_Invesal(100);
			Arm_Ctrl_Info.Servo_4 = 120;
			Arm_Ctrl_Info.Servo_3 = -130;
			Arm_Ctrl_Info.Servo_2 = -40;
			osDelay (250);
			Arm_Invesal(200);
			Arm_Ctrl_Info.Servo_1 = -30;
			osDelay (300);
			Arm_Ctrl_Info.Servo_3 = -60;
			osDelay (200);
			//目标点
//			Arm_Locus_Info.x=250;//265;
//			Arm_Locus_Info.y=0;//150;
//			Arm_Locus_Info.z=80;//80;
//			Arm_Ctrl_Info.Servo_2=25;
//			osDelay(Arm_Ctrl_Info.Speed_all);
//			Arm_Ctrl_Info.Servo_1=15;
//			Arm_Ctrl_Info.Servo_3=-40;
//			osDelay (500);
			
			Arm_Locus_Info.x=250;//265;
			Arm_Locus_Info.y=0;//150;
			Arm_Locus_Info.z=60;//80;
			Inverse_Kinematic(Arm_Locus_Info.x,Arm_Locus_Info.y,Arm_Locus_Info.z);
			Arm_Ctrl_Info.Servo_2 = -35;
			osDelay (100);
			Arm_Ctrl_Info.Servo_2 = Arm_Theta_Info.theta_2;
			Arm_Ctrl_Info.Servo_1 = Arm_Theta_Info.theta_1;
			Arm_Ctrl_Info.Servo_3 = Arm_Theta_Info.theta_3;
//			Arm_Ctrl_Info.Servo_1 = theta_0 [1]+72.5674;
//			Arm_Ctrl_Info.Servo_2 = theta_0 [2]-61.4246;
//			Arm_Ctrl_Info.Servo_3 = theta_0 [3]-43.9920;
			osDelay (300);
			Arm_Locus_Info.x=250;//265;
			Arm_Locus_Info.y=0;//150;
			Arm_Locus_Info.z=50;//80;
      Inverse_Kinematic(Arm_Locus_Info.x,Arm_Locus_Info.y,Arm_Locus_Info.z);
			Arm_Ctrl_Info.Servo_1 = Arm_Theta_Info.theta_1;
			Arm_Ctrl_Info.Servo_2 = Arm_Theta_Info.theta_2;
			Arm_Ctrl_Info.Servo_3 = Arm_Theta_Info.theta_3;
//			Arm_Ctrl_Info.Servo_1 = theta_0 [1]+78.3805;
//			Arm_Ctrl_Info.Servo_2 = theta_0 [2]-57.9726;
//			Arm_Ctrl_Info.Servo_3 = theta_0 [3]-46.3531;
			Arm_Invesal(200);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(400);
			//过渡点
      Arm_Move_Transition_B(200,0,90);
			Arm_Ctrl_Info.Servo_0 = 30;
			osDelay (200);
			Arm_Ctrl_Info.Servo_2 = 60;
			osDelay (250);
      Arm_Move_Back();
			osDelay(100);
			break;
		}
		case 6:{
			Arm_Invesal(100);
			Arm_Ctrl_Info.Servo_4 = 120;
			Arm_Ctrl_Info.Servo_3 = -130;
			Arm_Ctrl_Info.Servo_2 = -40;

			osDelay (250);
			Arm_Invesal(200);
			Arm_Ctrl_Info.Servo_1 = 10;
			osDelay (200);
			
			Arm_Ctrl_Info.Servo_2 = -15;
			Arm_Ctrl_Info.Servo_3 = -110;
			osDelay (200);
			
			Arm_Ctrl_Info.Servo_0 =130;
			osDelay (200);
			Arm_Ctrl_Info.Servo_3 = -90;
			Arm_Ctrl_Info.Servo_1 = 5;
			osDelay (200);
			Arm_Ctrl_Info.Servo_4 = 85;
			Arm_Ctrl_Info.Servo_3 = -70;
			Arm_Ctrl_Info.Servo_2 = 0;

			//目标点
			Arm_Invesal(200);
			Arm_Locus_Info.x=250;//265;
			Arm_Locus_Info.y=-150;//150;
			Arm_Locus_Info.z=60;//80;
      Arm_Move_Front();
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(200);
			//准备到过渡点
			Arm_Ctrl_Info.Servo_2 = 60;
			Arm_Ctrl_Info.Servo_3 = -20;
			osDelay (200);
			//过渡点
			Arm_Move_Transition_B(120,-120,100);
			osDelay (200);
			//弯折1、2、3号舵机
			Arm_Ctrl_Info.Servo_2 = 0;
			Arm_Ctrl_Info.Servo_1 = -120;
			Arm_Ctrl_Info.Servo_3 = -90;
			osDelay (400);
      Arm_Move_Back();
			osDelay(100);
			break;
		}
/*顺序场地*/
#if !Field_Inverse
/*粗加工区 放物料 7,8,9*/
		case 7:{
			//位置7，粗加工区
			Arm_Invesal(300);//300
			Arm_Ctrl_Info.Speed_2 = 400;
			//过渡点
			Arm_Ctrl_Info.Servo_0=118;
      Arm_Move_Transition_F(176-30,-155+15,80+70);
			//目标点位
			Arm_Locus_Info.x=170;//不交换场地坐标172     167
			Arm_Locus_Info.y=-153.6;//不交换场地坐标-155    -153.1
			Arm_Locus_Info.z=72.5;//不交换场地坐标76       72.5
			Arm_Invesal(600);
			Arm_Ctrl_Info.Speed_3 = 500;
      Arm_Move_Front();
			osDelay (450);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(300);//300
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-30,Arm_Locus_Info.y+15,Arm_Locus_Info.z+70);
      Arm_Move_Back();
			break;
		}
		case 8:{
			//位置8，粗加工区
			Arm_Invesal(400);//300
			//过渡点
      Arm_Move_Transition_F(176-20,-6,80+70);
			osDelay(100);
			//目标点位
			Arm_Locus_Info.x=172.8;//交换场地前坐标174      172
			Arm_Locus_Info.y=-9.3;//交换场地前坐标-7.5      -8
			Arm_Locus_Info.z=76;//交换场地前坐标76
			Arm_Invesal(600);
			Arm_Ctrl_Info.Speed_3 = 500;
      Arm_Move_Front();
			osDelay (400);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(300);//300
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-20,Arm_Locus_Info.y,Arm_Locus_Info.z+70);
			Arm_Ctrl_Info.Speed_2 = 200;
      Arm_Move_Back();
			break;
		}
		case 9:{
			//位置9，粗加工区
			osDelay(300);
			//过渡点
			osDelay(150);
			Arm_Invesal(400);//300
			Arm_Ctrl_Info.Speed_2 = 500;
			Arm_Ctrl_Info.Servo_0=50;
      Arm_Move_Transition_F(170-40,115-20,80+70);
			osDelay(100);
			//目标点位
			Arm_Locus_Info.x=165.7;//不交换场地坐标167           //165
			Arm_Locus_Info.y=156;//不交换场地坐标154.5          //156
			Arm_Locus_Info.z=76;//不交换场地坐标76
			Arm_Invesal(600);
			Arm_Ctrl_Info.Speed_3 = 500;
      Arm_Move_Front();
			osDelay (400);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(300);//300
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-30,Arm_Locus_Info.y-15,Arm_Locus_Info.z+70);
      Arm_Move_Back();
			break;
		}
#endif
		
/*翻转场地*/
#if Field_Inverse
/*粗加工区 放物料 7,8,9*/
		case 7:{
			//位置7，粗加工区
			Arm_Invesal(300);//300
			Arm_Ctrl_Info.Speed_2 = 400;
			//过渡点
			Arm_Ctrl_Info.Servo_0=118;
      Arm_Move_Transition_F(176-30,-155+15,80+70);
			//目标点位
			Arm_Locus_Info.x=166.3;//不交换场地坐标172     166
			Arm_Locus_Info.y=-154;//不交换场地坐标-155    -153.5
			Arm_Locus_Info.z=72.5;//不交换场地坐标76       74.5
			Arm_Invesal(600);
			Arm_Ctrl_Info.Speed_3 = 500;
      Arm_Move_Front();
			osDelay (450);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(300);//300
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-30,Arm_Locus_Info.y+15,Arm_Locus_Info.z+70);
      Arm_Move_Back();
			break;
		}
		case 8:{
			//位置8，粗加工区
			Arm_Invesal(400);//300
			//过渡点
      Arm_Move_Transition_F(176-20,-6,80+70);
			osDelay(100);
			//目标点位
			Arm_Locus_Info.x=172;//交换场地前坐标174      172
			Arm_Locus_Info.y=-8;//交换场地前坐标-7.5      -8
			Arm_Locus_Info.z=76;//交换场地前坐标76
			Arm_Invesal(600);
			Arm_Ctrl_Info.Speed_3 = 500;
      Arm_Move_Front();
			osDelay (400);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(300);//300
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-20,Arm_Locus_Info.y,Arm_Locus_Info.z+70);
			Arm_Ctrl_Info.Speed_2 = 200;
      Arm_Move_Back();
			break;
		}
		case 9:{
			//位置9，粗加工区
			osDelay(300);
			//过渡点
			osDelay(150);
			Arm_Invesal(400);//300
			Arm_Ctrl_Info.Speed_2 = 500;
			Arm_Ctrl_Info.Servo_0=50;
      Arm_Move_Transition_F(170-40,115-20,80+70);
			osDelay(100);
			//目标点位
			Arm_Locus_Info.x=165;//不交换场地坐标167           //165
			Arm_Locus_Info.y=156;//不交换场地坐标154.5          //156
			Arm_Locus_Info.z=76;//不交换场地坐标76
			Arm_Invesal(600);
			Arm_Ctrl_Info.Speed_3 = 500;
      Arm_Move_Front();
			osDelay (400);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(300);//300
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-30,Arm_Locus_Info.y-15,Arm_Locus_Info.z+70);
      Arm_Move_Back();
			break;
		}
#endif
/*粗加工区 取物料 10,11,12*/
		case 10:{
			//粗加工区7+3，取
			Arm_Invesal(200);//300
			//过渡点
      Arm_Move_Transition_F(176-30,-155+20,80+75);
			//目标点位
			Arm_Locus_Info.x=166;//172另一场地
			Arm_Locus_Info.y=-153;//-155
			Arm_Locus_Info.z=74;//76
			Arm_Invesal(400);
			Arm_Ctrl_Info.Speed_2 = 300;
      Arm_Move_Front();
			osDelay(200);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(200);//300
			Arm_Ctrl_Info.Speed_2 = 300;
      Arm_Move_Transition_B(Arm_Locus_Info.x-30,Arm_Locus_Info.y+15,Arm_Locus_Info.z+70);
      Arm_Move_Back();
			break;
		}
		case 11:{
			//粗加工区8+3，取
			Arm_Invesal(200);//300
			//过渡点
			Arm_Ctrl_Info.Servo_4=110;
      Arm_Move_Transition_F(175-30,-6,80+70);
			//目标点位
			Arm_Locus_Info.x=174;//263
			Arm_Locus_Info.y=-7.5;//9
			Arm_Locus_Info.z=76;//80
			Arm_Invesal(400);
			Arm_Ctrl_Info.Speed_3 = 300;
      Arm_Move_Front();
			osDelay(200);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(200);//300
			Arm_Ctrl_Info.Speed_3 = 300;
      Arm_Move_Transition_B(Arm_Locus_Info.x-30,Arm_Locus_Info.y,Arm_Locus_Info.z+70);
      Arm_Move_Back();
			break;
		}
		case 12:{
			//粗加工区9+3，取
			//过渡点
			Arm_Invesal(200);//200
      Arm_Move_Transition_F(167-30,154-40,80+70);
			Arm_Locus_Info.x=167;//265
			Arm_Locus_Info.y=154.5;//144
			Arm_Locus_Info.z=76;//80
      Arm_Invesal(400);
			Arm_Ctrl_Info.Speed_2 = 300;
			Arm_Move_Front();
			osDelay(200);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(200);//300
			Arm_Ctrl_Info.Speed_2 = 300;
      Arm_Move_Transition_B(Arm_Locus_Info.x-30,Arm_Locus_Info.y-25,Arm_Locus_Info.z+70);
      Arm_Move_Back();
			break;
		}
/*半成品放 13,14,15*/
		case 13:{
			osDelay(100);
			//半成品区13，放  242,-155,32
			Arm_Invesal(300);
			//过渡点
      Arm_Move_Transition_F(240-48,-160+32,32+70);
			//目标点位
			Arm_Locus_Info.x=242.5;//240.5顺序场地      241.5
			Arm_Locus_Info.y=-154.8;//-159.5           -154.3
			Arm_Locus_Info.z=25;   //26.5;
			Arm_Invesal(500);
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Front();
			osDelay (400);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-60,Arm_Locus_Info.y+40,Arm_Locus_Info.z+70);
      Arm_Move_Back();
			break;
		}
		case 14:{
			//半成品区14，放 250,-5,32
			Arm_Invesal(300);
			//过渡点
      Arm_Move_Transition_F(250-50,-3,32+70);
			//目标点位
			Arm_Locus_Info.x=247.8;//249;顺序场地   247.5
			Arm_Locus_Info.y=-11;//-11;           -10
			Arm_Locus_Info.z=26;//26;
			Arm_Invesal(500);
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Front();
			osDelay (400);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-50,Arm_Locus_Info.y,Arm_Locus_Info.z+70);
      Arm_Move_Back();
			break;
		}
		case 15:{
			//半成品区15，放
			Arm_Invesal(300);
			//过渡点
      Arm_Move_Transition_F(252-50,143-40,32+70);
			//目标点位
			Arm_Locus_Info.x=240.5;//242.2;   240.2
			Arm_Locus_Info.y=158;//157.4;     158
			Arm_Locus_Info.z=24;//24;
			Arm_Invesal(500);
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Front();
			osDelay (400);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-50,Arm_Locus_Info.y-40,Arm_Locus_Info.z+70);
      Arm_Move_Back();
			break;
		}
/*码垛16,17,18，分别加3*/
		case 16:{
			osDelay(100);
			//半成品区13，码垛
			Arm_Invesal(350);
			//过渡点
      Arm_Move_Transition_F(242-60,-155+40,112+50);
			//目标点位
			Arm_Locus_Info.x=242.5;//243.5;顺序场地
			Arm_Locus_Info.y=-154.8;//-155
			Arm_Locus_Info.z=101.5;//101.5
			Arm_Invesal(600);
			Arm_Ctrl_Info.Speed_2 = 500;
      Arm_Move_Front();
			osDelay (300);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-60,Arm_Locus_Info.y+40,Arm_Locus_Info.z+50);
      Arm_Move_Back();
			break;
		}
		case 17:{
			//半成品区17，码垛
			Arm_Invesal(350);
			//过渡点
      Arm_Move_Transition_F(250-60,-5,112+60);
			//目标点位
			Arm_Locus_Info.x=247.8;//249;      250.5
			Arm_Locus_Info.y=-11;//-11;      -9.5
			Arm_Locus_Info.z=112;//112;
			Arm_Invesal(600);
			Arm_Ctrl_Info.Speed_2 = 400;
			Arm_Ctrl_Info.Speed_3 = 400;
      Arm_Move_Front();
			osDelay (300);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-50,Arm_Locus_Info.y,Arm_Locus_Info.z+50);
      Arm_Move_Back();
			break;
		}
		case 18:{
			osDelay(600);
			//半成品区18，码垛
			Arm_Invesal(350);
			//过渡点
      Arm_Move_Transition_F(252-50,152-20,112+60);
			//目标点位
			Arm_Locus_Info.x=240.5;//242.2
			Arm_Locus_Info.y=158;//157.4
			Arm_Locus_Info.z=100;//100;
			Arm_Invesal(600);
			Arm_Ctrl_Info.Speed_2 = 500;
      Arm_Move_Front();
			osDelay (300);
			Arm_Ctrl_Info.Servo_4 = Arm_Theta_Info.theta_4;
			osDelay(Arm_Ctrl_Info.Speed_4+100);
			//过渡点
			Arm_Invesal(200);
			Arm_Ctrl_Info.Speed_2 = 400;
      Arm_Move_Transition_B(Arm_Locus_Info.x-50,Arm_Locus_Info.y-40,Arm_Locus_Info.z+50);
      Arm_Move_Back();
			break;
		}
	}
}

/*机械臂运动顺序判断*/
void Arm_Move_Path(int inner,int outer){
  if((outer >= 7 && outer <=9) || (outer>=13)){
		//从物料盘取放至于场地
    Arm_Move_Inner(inner,outer);
    osDelay(400);
    Arm_Move_Outer(outer);
    osDelay(400);	 
		Arm_Order_Info.inner=0;
		Arm_Order_Info.outer=0;
  }
  else if((outer <=6 && outer >=0) || (outer >=10 && outer <=12)){
		//从场地取放置于物料盘
		Arm_Move_Outer(outer);
		osDelay(400);
		Arm_Move_Inner(inner,outer);
		osDelay(400);
		Arm_Order_Info.inner=0;
		Arm_Order_Info.outer=0;
  }
}

void Arm_Move_Init(void){
//	Arm_Ctrl_Info.Servo_0 = 100.0f;
//	osDelay(1000);
//	Arm_Ctrl_Info.Servo_4 = 135.0f;
	Arm_Move_Path(Arm_Order_Info.inner,Arm_Order_Info.outer);
//调参函数，退出debug后需注释
//	Arm_xyz(Arm_Locus_Info.x_3,Arm_Locus_Info.y_3,Arm_Locus_Info.z_3);
	Arm_Ctrl_Info.Status = Arm_Ctrl_Debug;
}

