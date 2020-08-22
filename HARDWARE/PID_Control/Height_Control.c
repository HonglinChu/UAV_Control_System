#include "Height_Control.h"
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define ABS(x) ( (x)>0?(x):-(x) )
extern float   Thr_Weight;
u8 ultra_start_f =0;
u8 height_start_f=0;
extern float vx,vy,vz;
float wz_speed,wz_speed_old;
float Actual_Acc;//
float ultra_speed,ultra_distance,ultra_distance_old,ultra_delta,ultra_ctrl_out,ultra_dis_lpf;
//高度控制函数
PID Height_PID1;//高度外环
PID Height_PID2;//高度内环
//高度控制函数
void Height_Control(float T,float thr)
{   
	static u8 cnt=0;
	//相对加速度
	if(height_start_f==0)//第一次进入定高函数
	{
	   Actual_Acc=0; 
	}
	Actual_Acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *( (vz *ACC.Z + vx *ACC.X + vy*ACC.Y - 8192 ) - Actual_Acc);
	cnt++;
	cnt = cnt%10;
	if(cnt == 0)//20ms控制一次
	{
	   Ultra_In_Control(0.02f,thr,0.4f*ultra_ctrl_out,ultra_speed);
	}
	if(ultra_start_f == 1)//超声波完成了一次更新
	{
	   Ultra_Out_Control(0.1f,thr);//超声波周期100ms
	   ultra_start_f = 0;//更新标志清除
	}
}

//高度控制外环
void Ultra_Out_Control(float T,float thr)
{     
	  static float exp_height,height_err_old;
	  float        exp_speed,ultra_sp_tmp,ultra_dis_tmp,height_err;
	  //注意这个期望速度和高度有关系（一键起飞，最大高度2m）
	 if(height_start_f==0)//第一次进入定高模式期望高度等于当前高度,相关积分都要清零
	 {
	   exp_height=ultra_distance;
	   Height_PID1.IOut=0;
	   height_err_old=0;
	   ultra_dis_lpf=0;
	   ultra_speed=0;
	   height_start_f=1;
	 }
	 //油门在1400到1600之间期望速度为0
	 exp_speed=LIMIT((300*my_deathzoom_2(thr-500,100)/200.0f),-300,300);
	 if(exp_height>2000)//期望高度不能超过2m
	 { 
		exp_height=500;
		if(exp_speed>0)
		{
		  exp_speed=0;
		}		
	 }
	 else if(exp_height<200)//最小期望高度20cm
	 {
		  exp_height=200;//20cm
		  if(exp_speed<0)
		  {
			exp_speed=0;
		  }
	 }
    exp_height+=exp_speed*T;//最大期望高度1.5m
	if(thr<100)//如果油门小于100，期望高度开始下降(一键降落)
	{  
	  //高度逐渐减小
	  exp_height += ( 1 / ( 1 + 1 / ( 0.2f *3.14f *T ) ) ) *( -exp_height);
//	  if(exp_height<=200)//20cm
//	  {
//	    exp_height=0;
//	  }
	}
	ultra_sp_tmp = Moving_Median(0,5,ultra_delta/T); //ultra_delta/T;
    //对速度进行低通滤波
    if( ABS(ultra_sp_tmp) < 100)
    {
        ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
    }
    else
    {
        ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
    }
    //超声波测量的距离进行滑动滤波
	ultra_dis_tmp = Moving_Median(1,5,ultra_distance);
    //对超声波测得的距离进行低通滤波
    if(ABS(ultra_dis_tmp - ultra_dis_lpf) < 100)
    {
        ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
    }
    else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 200 )
    {

        ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
    }
    else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 400 )
    {
        ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
    }
    else
    {
        ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 0.6f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
    }
	//期望高度是由油门给出的，，，期望高度减去低通滤波之后的高度=高度误差
	height_err =Height_PID1.P*(exp_height - ultra_dis_lpf);
	//误差积分
	Height_PID1.IOut += Height_PID1.I *height_err *T;
	//积分限制幅度300*油门权重
	Height_PID1.IOut = LIMIT(Height_PID1.IOut,-Thr_Weight *300,Thr_Weight *300);
	//不完全微分
	Height_PID1.DOut = Height_PID1.D *(0.6f *(-wz_speed*T) + 0.4f *(height_err - height_err_old) );
	Height_PID1.Out  = LIMIT(( height_err + Height_PID1.IOut + Height_PID1.DOut),-500,500);
    //积分限制幅度
	ultra_ctrl_out=Height_PID1.Out;
	height_err_old = height_err;
}

//超声波定高模式内环
//定高模式内环速度控制，参数T，THR（0-1000），期望Z轴速度和实际Z轴速度   速度控制环的周期为20ms
void Ultra_In_Control(float T,float thr,float exp_z_speed,float h_speed)
{
	static float thr_lpf;
	float height_thr,Actual_Acc_mms2,speed_err;
	static float hc_acc_i,wz_speed_0,wz_speed_1;
    //高度油门限制幅度
	if(height_start_f==1)//第一次进入这个函数
	{
	   hc_acc_i=0;
	   wz_speed_0=0;
	   wz_speed_1=0;
	   thr_lpf=thr;     //可以考虑将当前油门值直接赋值防止切换到定高模式的时候吊高
	  Height_PID2.IOut=0;
	}
	//为了防止刚刚计入定高的时候直接降落，
	height_thr = LIMIT( 2 * thr , 0, 600 );//高度油门是0-600//0-700
    //高度油门低通滤波
	thr_lpf += ( 1 / ( 1 + 1 / ( 0.5f *3.14f *T ) ) ) *( height_thr - thr_lpf );
	//wz_speed的更新有问题？？？？？？？？？？？？？？？？？？？
	//加速度数据和超声波数据进行融合。。。。开始
	Actual_Acc_mms2 = (Actual_Acc/8192.0f) *10000 + hc_acc_i;//9.8m/S^2=10m/S^2=10000mm/S^2
	wz_speed_0 += my_deathzoom((Actual_Acc_mms2) ,100) *T;//if acc>100 则(acc-100)*T   else if acc<-100,则(acc+100)*T
	//加速度误差积分
	hc_acc_i += 0.4f *T *((wz_speed - wz_speed_old)/T - Actual_Acc_mms2 );//加速度误差=加加速度 积分ki=0.4*T
	//加速度误差限制幅度
	hc_acc_i = LIMIT( hc_acc_i, -500, 500 );
	//对超声波测得的速度进行低通滤波
	wz_speed_0 += ( 1 / ( 1 + 1 / (0.1f *3.14f *T ) ) ) *(h_speed - wz_speed_0) ;
	wz_speed_1 = wz_speed_0;//经过滤波后的超声波测得的速度单位mm/s
	if(ABS(wz_speed_1)<50)//50mm/s 速度也不小了啊
	{
		wz_speed_1 = 0;
	}
	wz_speed_old = wz_speed;
	wz_speed = wz_speed_1;  //经过滤波后的超声波测得的速度等于下一次的速度
	//加速度计数据和超声波数据进行融合结束
	speed_err = Height_PID2.P *( exp_z_speed - wz_speed );
	Height_PID2.DOut= 0.002f/T *10*Height_PID2.D * (-Actual_Acc_mms2)*T;//(Height_PID2_v.err - Height_PID2_v.err_old);
	//Height_PID2_v.err_i += Height_PID2.ki *Height_PID2_v.err *T;
	Height_PID2.IOut+= Height_PID2.I *Height_PID2.P*(exp_z_speed - h_speed)*T;
    //油门权重在（0~1）
	Height_PID2.IOut = LIMIT(Height_PID2.IOut,-Thr_Weight *300,Thr_Weight *300);
	Height_PID2.Out = thr_lpf + Thr_Weight *LIMIT((Height_PID2.P *exp_z_speed + speed_err + Height_PID2.DOut+ Height_PID2.IOut),-300,300); 
}

float my_deathzoom(float x,float zoom)
{
    float t;
    if(x>0)
    {
        t = x - zoom;
        if(t<0)
        {
            t = 0;
        }
    }
    else
    {
        t = x + zoom;
        if(t>0)
        {
            t = 0;
        }
    }
    return (t);
}


//设置死区函数
float my_deathzoom_2(float x,float zoom)
{
    float t;

    if( x> -zoom && x < zoom )
    {
        t = 0;
    }
    else
    {
        t = x;
    }
    return (t);
}

//滑动滤波函数
#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  2
#define MED_FIL_ITEM_int  2
float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
int med_filter_tmp_int[MED_FIL_ITEM_int][MED_WIDTH_NUM ];
u8 med_fil_cnt[MED_FIL_ITEM];
float Moving_Median(u8 item,u8 width_num,float in)
{
    u8 i,j;
    float t;
    float tmp[MED_WIDTH_NUM];

    if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
        return 0;
    else
    {
        if( ++med_fil_cnt[item] >= width_num )
            med_fil_cnt[item] = 0;
        med_filter_tmp[item][ med_fil_cnt[item] ] = in;
        for(i=0; i<width_num; i++)
            tmp[i] = med_filter_tmp[item][i];
        for(i=0; i<width_num-1; i++)
        {
            for(j=0; j<(width_num-1-i); j++)
            {
                if(tmp[j] > tmp[j+1])
                {
                    t = tmp[j];
                    tmp[j] = tmp[j+1];
                    tmp[j+1] = t;
                }
            }
        }
        return ( tmp[(u16)width_num/2] );
    }
}
