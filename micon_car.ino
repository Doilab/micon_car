//専門実験・演習B ロボット制御２
//2021.10.07 Takahiro Doi
//以下3つのうちどれか一つだけを有効にすること
#define WEEK_1 //第1週の実験のときはここを有効にする
//#define WEEK_2 //第2週の実験のときはここを有効にする
//#define WEEK_3 //第3週の実験のときはここを有効にする

//第1週の基本的な機能確認処理はL158からのw1_sensor_test()を参照すること
//第2週の限界感度法による制御パラメータチューニングはL212からのw2_zn_tuning()を参照すること
//第3週のステップ応答とライントレースについては，L311からのw3_line_trace()を参照すること

#define MODE_P 1
#define MODE_PI 2
#define MODE_PID 3
#define MODE_ORIGINAL 4
#define SAMPLING_TIME_MSEC 1 //サンプリングタイム[msec]

int pot_data;
int sensor_L_data;
int sensor_R_data;
long time_now;
long time_start;
int pwm_1;//モータ１に与えるPWM
int pwm_2;//モータ２に与えるPWM
int led_pwm;//LEDに与えるPWM

//---------------------------------------------------
void update_status()
{
  pot_data = analogRead(A0);
  sensor_L_data = analogRead(A1);
  sensor_R_data = analogRead(A2);
  time_now = millis()-time_start;
}
//---------------------------------------------------
void print_status()
{
//  Serial.println(" ");
  Serial.print(time_now);
  Serial.print(",");
  Serial.print(pot_data);
  Serial.print(",");
  Serial.print(sensor_L_data);
  Serial.print(",");
  Serial.print(sensor_R_data);
  Serial.print(",   ");
  Serial.print(pwm_1);
  Serial.print(",   ");
  Serial.print(pwm_2);
  //Serial.print(",   ");
  //Serial.print(",LED:");
  //Serial.print(led_pwm);
  Serial.println(",   ");
  
}
//---------------------------------------------------
void drive_motor(int no, int duty)
{
  int absduty=abs(duty);
  int cw_ch,ccw_ch;//方向指令チャンネル
  int pwm_ch;//モータのチャンネル
int stnby_ch;//スタンバイポート
  
  if(no==1)
  {
    cw_ch=2;
    ccw_ch=4;
    pwm_ch=3;
    stnby_ch=10;
    pwm_1=duty;
  }
  else if(no==2)
  {
    cw_ch=7;
    ccw_ch=5;
    pwm_ch=6;
    stnby_ch=11;
    pwm_2=duty;
  }
  else
  {
    return;
  }
    if(duty>0)
//    if(duty<0)
    {
    digitalWrite(cw_ch,HIGH);
    digitalWrite(ccw_ch,LOW);
    }
    else
    {
    digitalWrite(cw_ch,LOW);
    digitalWrite(ccw_ch,HIGH);
    }
//    return;//デバッグ用モータ停止

    if(absduty>255)absduty=255;//最大値でクリップ
    analogWrite(pwm_ch,absduty);//PWM発生
    digitalWrite(stnby_ch,HIGH);//ロック解除
  

}
//---------------------------------------------------
void set_LED(int no, int pwm_in)
{
  int abspwm=abs(pwm_in);
  if(abspwm>255)abspwm=254;//最大値でクリップ
  
  if(no==1)analogWrite(A4,abspwm);
  if(no==2)analogWrite(A5,abspwm);
}
//---------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  //初期化

  int i;

  pinMode(2, OUTPUT);   // output CW1
  pinMode(4, OUTPUT);   // output CCW1
  pinMode(7, OUTPUT);   // output CW2
  pinMode(5, OUTPUT);   // output CCW2
  pinMode(8, INPUT_PULLUP);   // 入力ピンに設定　SW1
  pinMode(9, INPUT_PULLUP);   // 入力ピンに設定　SW2
  time_start=millis();

  Serial.begin(115200);
//Initialize serial and wait for port to open:
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // prints title with ending line break
  Serial.println("=== KIT Micon Car Control System ===");
  #ifdef WEEK_1 
  Serial.println("=== WEEK_1 defined ===");
  #endif
  #ifdef WEEK_2 
  Serial.println("=== WEEK_2 defined ===");
  #endif
  #ifdef WEEK_3 
  Serial.println("=== WEEK_3 defined ===");
  #endif
  //LEDの点滅
  for(i=0;i<3;i++)
  {
    set_LED(1,130);
    set_LED(2,130);
    delay(100);
    set_LED(1,0);
    set_LED(2,0);
    delay(100);
  }
}
//---------------------------------------------------
//　第1週　センサデータと基本機能の確認
//---------------------------------------------------
void w1_sensor_test(void)
{
  //テスト用関数
int motor_no=0;
int i;

//スイッチ状態の確認
if(digitalRead(8)==LOW)
{
  Serial.println("SW1(D8)-L");
  motor_no=1;
  set_LED(1,150);
}
else if(digitalRead(9)==LOW)
{
  Serial.println("SW2(D9)-L");
  motor_no=2;
  set_LED(2,150);
}

//センサ状態表示
  update_status();
  print_status();
  delay(50);


//  for(motor_no=1; motor_no<=2; motor_no++)
  if((motor_no==1)||(motor_no==2))
  {
   Serial.print("motor-");
   Serial.println(motor_no);
  drive_motor(motor_no,0);
  delay(100);
  drive_motor(motor_no,100);
  delay(500);
  drive_motor(motor_no,0);
  delay(100);
  drive_motor(motor_no,-100);
  delay(500);
  drive_motor(motor_no,0);
  delay(100);
   Serial.println("done");
  }

  //LED消灯
    set_LED(1,0);
    set_LED(2,0);

}
//---------------------------------------------------
//　第2週　限界感度法を実行する関数
//　シリアル通信の表示は使わず，持続振動するかどうかを確認する．
//  振動の周期をオシロスコープを使って計測する
//---------------------------------------------------
void w2_zn_tuning(void)
{
//限界感度法によるチューニング
//変数定義
  char stop_flag=0;
  int mode;
  double kp,ti,td;//制御パラメータ
  double e;//error（制御偏差）
  double de_dt;//errorの時間微分
  double e_s=0;//errrorの時間積分
  long dt_msec=SAMPLING_TIME_MSEC;//サンプリングタイムmsec
  double dt=(double)SAMPLING_TIME_MSEC/1000.0;//サンプリングタイム[sec]
  double e_prev=0;//1周期前のerror
  double m_pwm=0;//ステアリングモータへの出力
  long time_prev;//前回状態表示した時刻

//制御実行前の準備
mode=MODE_P;//P補償

if(mode==MODE_P)//P補償
{
   kp=0.005;//☆☆☆☆☆☆　ここを調整する　☆☆☆☆☆☆☆
}



//制御パラメータ表示
  Serial.println(" ");
  Serial.println("===Start ZN tuning =====");
  if(mode==MODE_P)Serial.print("P-control ");
  else Serial.print("?-control ");
  Serial.print("(kp)= ");
  Serial.print(kp,4);//小数点以下4桁まで表示
  Serial.println(" ");
  
  delay(1000);//実行前待機

  //制御処理（無限ループ）
  //--------- whileループここから ------------
  while(1)
  {
    update_status();//センサ状態更新
    if((time_now-time_prev)>50)//50msecに1度だけ状態表示
    {
      print_status();
      time_prev=time_now;
    }
  
    //スイッチによる停止処理
    if(digitalRead(8)==LOW)stop_flag=1;
    if(digitalRead(9)==LOW)stop_flag=1;
    if(stop_flag==1)break;//ループから抜ける


    //制御側
    e = (double)sensor_L_data-(double)sensor_R_data;

    if(mode==MODE_P) m_pwm=kp*e; //p補償

    //制御状態表示用LED点灯
    led_pwm=(int)m_pwm * 10;
    set_LED(1,0);//LED1 一旦消灯
    set_LED(2,0);//LED2 一旦消灯

    if(led_pwm > 0)//モータ正方向駆動（LED1点灯）
    {
      set_LED(1,130);
      set_LED(2,0);
    }

    if(led_pwm < 0)//モータ逆方向駆動（LED2点灯）
    {
      set_LED(1,0);
      set_LED(2,130);
    }

    //モータ駆動
    drive_motor(1,m_pwm*255);//ステアリング制御
    drive_motor(2,0);//タイヤ駆動モータ制御（停止）
 
    //サンプリングタイム処理
    delay(dt_msec);//サンプリングタイム休む
  }
  //------- whileループここまで -----------


  //ループ外（停止処理）
   Serial.println(" ");
   Serial.println("===Stop ZN tuning =====");
    drive_motor(1,0);//モータ１停止
    drive_motor(2,0);//モータ２停止
    set_LED(1,0);//LED1 一旦消灯
    set_LED(2,0);//LED2 一旦消灯
    delay(1000);
}
//---------------------------------------------------
//　第3週　ステップ応答，ライントレースの関数
//  シリアル通信のログをグラフ化する．
//---------------------------------------------------
void w3_line_trace(void)
{
//変数定義
  char stop_flag=0;
  int mode;
  double kp,ti,td;//制御パラメータ
  double e;//error（制御偏差）
  double de_dt;//errorの時間微分
  double e_s=0;//errrorの時間積分
  long dt_msec=SAMPLING_TIME_MSEC;//サンプリングタイムmsec
  double dt=(double)SAMPLING_TIME_MSEC/1000.0;//サンプリングタイム[sec]
  double e_prev=0;//1周期前のerror
  double m_pwm=0;//ステアリングモータへの出力（0～1）
  double m_pwm2=0.5;//タイヤ駆動用モータのPWM（一定値．0～1）ステップ応答の実験の際にはゼロにしておく．
  long time_prev;//前回状態表示した時刻

//制御実行前の準備
//補償方式を選ぶ
mode=MODE_P;//P補償
//mode=MODE_PID;//PID補償
//mode=MODE_PI;//PI補償
//mode=MODE_ORIGINAL;//独自の補償

if(mode==MODE_P)//P補償
{
   kp=0.00531*0.5;//限界感度法の値に基づいて設定する
}
if(mode==MODE_PI)//PI補償
{
   kp=0.00531*0.45;//限界感度法の値に基づいて設定する
   ti=0.253/1.2;//限界感度法の値に基づいて設定する
}
if(mode==MODE_PID)//PID補償
{
    kp=0.00531*0.6;//限界感度法の値に基づいて設定する
    ti=0.253*0.5;//限界感度法の値に基づいて設定する
    td=0.253/8;//限界感度法の値に基づいて設定する
}
if(mode==MODE_ORIGINAL)//各班独自の補償
{
  //ここに独自に考えた補償パラメータを書く
  ;
}


//制御パラメータ表示
  Serial.println(" ");
  Serial.println("===Start Line Trace =====");
  if(mode==MODE_P)Serial.print("P-control ");
  else if(mode==MODE_PI)Serial.print("PI-control ");
  else if(mode==MODE_PID)Serial.print("PID-control ");
  else Serial.print("?-control ");
  Serial.print("(kp,ti,td)= ");
  Serial.print("(");
  Serial.print(kp,4);
  Serial.print(",");
  Serial.print(ti,4);
  Serial.print(",");
  Serial.print(td,4);
  Serial.println(")");
  Serial.println("time, pot, L, R, PWM1, PWM2");
  
  delay(1000);//実行前待機

time_now=millis();
time_prev=time_now;

  //制御処理（無限ループ）
  //--------- whileループここから ------------
  while(1)
  {
    update_status();//センサ状態更新
    if((time_now-time_prev)>50)//50msecに1度だけ状態表示
    {
      print_status();
      time_prev=time_now;
    }
  
    //スイッチによる停止処理
    if(digitalRead(8)==LOW)stop_flag=1;
    if(digitalRead(9)==LOW)stop_flag=1;
    if(stop_flag==1)break;//ループから抜ける


    //制御側
    e = (double)sensor_L_data-(double)sensor_R_data;
    de_dt=(e-e_prev)/dt;//微分
    e_s+=e*dt;//積分
    e_prev=e;//次の微分のためのパラメータ保存

    if(mode==MODE_P) m_pwm=kp*e; //p補償
    if(mode==MODE_PI) m_pwm=kp*(e+(e_s/ti));//PI補償．tiがゼロにならないように注意．
//    if(mode==MODE_PID) m_pwm=kp*(e+(e_s/ti)+e_s*td);//PID補償．tiがゼロにならないように注意．
    if(mode==MODE_PID) m_pwm=kp*(e+(e_s/ti)+de_dt*td);//PID補償．tiがゼロにならないように注意．181116修正
    if(mode==MODE_ORIGINAL)
    {
      //ここに独自に考えた制御則を書く
      ;
    }

    //制御状態表示用LED点灯
    led_pwm=(int)m_pwm * 10;
    set_LED(1,0);//LED1 一旦消灯
    set_LED(2,0);//LED2 一旦消灯

    if(led_pwm > 0)//モータ正方向駆動（LED1点灯）
    {
      set_LED(1,130);
      set_LED(2,0);
    }

    if(led_pwm < 0)//モータ逆方向駆動（LED2点灯）
    {
      set_LED(1,0);
      set_LED(2,130);
    }

    //モータ駆動
    drive_motor(1,m_pwm*255);//ステアリング制御
    drive_motor(2,m_pwm2*255);//タイヤ駆動モータ制御（一定速度）
 
    //サンプリングタイム処理
    delay(dt_msec);//サンプリングタイム休む
  }
  //------- whileループここまで -----------


  //ループ外（停止処理）
   Serial.println(" ");
   Serial.println("===Stop Line Trace =====");
    drive_motor(1,0);//モータ１停止
    drive_motor(2,0);//モータ２停止
    set_LED(1,0);//LED1 一旦消灯
    set_LED(2,0);//LED2 一旦消灯
    delay(1000);
}

//---------------------------------------------------
void loop() {
//メインの処理
//L4～L6の#defineを確認してから実行すること．

//第1週のプログラム
#ifdef WEEK_1
w1_sensor_test();//センサテスト用関数（ループなし）
#endif

//スイッチで第2週，第3週のプログラム開始
//if((digitalRead(8)==LOW)&&(digitalRead(9)==LOW))//両方押した場合
if((digitalRead(8)==LOW)||(digitalRead(9)==LOW))//どちらか押した場合
{
#ifdef WEEK_2
  Serial.println(" zn_tuning ");
  w2_zn_tuning();//限界感度法によるチューニング（無限ループ）
#endif
#ifdef WEEK_3
  Serial.println(" line_trace ");
  w3_line_trace();//ライントレース処理（無限ループ）
#endif
  
}

}
