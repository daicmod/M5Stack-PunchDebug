// define must ahead #include <M5Stack.h>
#define M5STACK_MPU6886 
// #define M5STACK_MPU9250 
// #define M5STACK_MPU6050
// #define M5STACK_200Q

#include <M5Stack.h>
#define WIDTH 320
#define HEIGHT 240

/* 定数名が悪い、最初は描画範囲として使ってたけど配列に使ったためおかしくなった */
#define GRAPH_WIDTH 20
#define GRAPH_HEIGHT 200

int countPunch(double point);
double getNorm(double x, double y, double z);
double dxax(double point);
void setGraph(double point1, double point2);
void drawGraph();
void IRAM_ATTR onTimer();

/* 実質定数、グラフ描画用の補正係数 */
int K = (int)(WIDTH / GRAPH_WIDTH);


/*  タイマー割り込み設定定義  */
hw_timer_t * timer = NULL;  //タイマー変数
volatile SemaphoreHandle_t timerSemaphore;  //タイマー割り込みが発生したかどうかを表すセマフォ
volatile int interruptCounter;  //タイマーカウンター

int graphArray[320] = { 0 };
int graphArray2[320] = { 0 };

/* センサデータ */
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float temp = 0.0F;

// the setup routine runs once when M5Stack starts up
void setup(){

  // Initialize the M5Stack object
  M5.begin();
  /*
    Power chip connected to gpio21, gpio22, I2C device
    Set battery charging voltage and current
    If used battery, please call this function in your project
  */
  M5.Power.begin();
    
  M5.IMU.Init();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);

  /*  タイマー割り込み設定  */
  timerSemaphore = xSemaphoreCreateBinary();  //割り込みハンドラが呼ばれた際の通信のために利用するセマフォ(バイナリセマフォ)の作成
  timer = timerBegin(0, 80, true);  //タイマー番号0(0〜3)、分周比80(ESP32のクロック周波数は8MHz)、タイマー増減(true:増加,false:減少)
  timerAttachInterrupt(timer, &onTimer, true);  //タイマー変数指定:timer、タイマー割り込みする処理関:onTimer、割り込みタイプ(true:エッジ,false:レベル)
  timerAlarmWrite(timer, 10000, true);  //タイマー変数指定:timer、タイマー10msec割り込み(8000000(クロック周波数) / 80(分周比) = 1000000Hz(=1カウントで1μs))、自動再起動有無(true:有,false:無)
  timerAlarmEnable(timer);  //タイマー起動
  
}

// the loop routine runs over and over again forever
void loop() {  
  // 各センサの値を更新する:
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  M5.IMU.getAhrsData(&pitch,&roll,&yaw);
  M5.IMU.getTempData(&temp);

  /* 加速度ベクトルの大きさからパンチを判定する */
  double nowNorm = getNorm(accX,accY,accZ);
  double di = diff(nowNorm);
  int counter = countPunch(nowNorm);

  if(interruptCounter > 0) {  //タイマー割り込みされれば
    interruptCounter --; //割り込みカウントリセット
    /* 実行したい処理をここ以降に処理プログラムを追加 */
    /* デバッグ用、加速度がどう推移するのか見たかった */
    setGraph(nowNorm, di);
    drawGraph();
    
    M5.Lcd.setTextColor(WHITE , BLACK);
    M5.Lcd.setCursor(0, 220);
    M5.Lcd.printf("Punch : %d", counter);
    M5.Lcd.setCursor(150, 205);
    M5.Lcd.printf("Acc  : %.3f", nowNorm);  
    M5.Lcd.setTextColor(GREEN , BLACK);
    M5.Lcd.setCursor(150, 220);
    M5.Lcd.printf("Diff : %.3f", di);
  }
}

/*  タイマー割り込み処理関数  */
void IRAM_ATTR onTimer() {

  interruptCounter++; //割り込みカウント
  xSemaphoreGiveFromISR(timerSemaphore, NULL);  //割り込みフラグをリセット
  /* タイマー内で処理させたい処理があればここ以降に処理プログラムを追加 */

}

/* パンチの計測は大雑把にヒステリシスコンパレータ式で計測してます */
int countPunch(double point)
{    
  static int counter = 0;
  static double maxPoint = -1;
  static double minPoint = 9999;
  double highHis = 40; 
  double lowHis = 10;
  
  maxPoint = ( maxPoint < point) ? point : maxPoint;
  if(highHis < maxPoint)
  { /* 加速度の最大値がhighHisを超えたらパンチを検出 */
    minPoint = ( minPoint > point) ? point : minPoint;
    if(lowHis > minPoint)
    { /* 加速度がlowHisを下回ったらパンチの終わりと判定 */
      counter++;
      maxPoint = -1;      
    }
  }
  else
  { /* 判定に使ってた変数を初期化しておく */
    minPoint = 9999;
  }
  
  return counter;
}

/* 前回の入力との差分を返します。パンチの計測には使っていないけど、加速度の微分が特徴量に使えるかもと思い用意した */
double diff(double point)
{
  static double prepoint = 1;
  double res = point - prepoint;
  prepoint = point;
  
  return res;
}

/* グラフ描画用に配列を格納します。配列をグルグル回して格納していきます */
void setGraph(double point1, double point2)
{
  static int counter = 0;
  if (counter == GRAPH_WIDTH) 
    counter = 0;
  
  int h = GRAPH_HEIGHT;
  
  /* point1のグラフが小さいので2倍した、drawGraphで閾値を表示するなら2倍するのを忘れずに */
  h = GRAPH_HEIGHT - (int)(2*point1);
  if(h < 0) h = 0;
  graphArray[counter] = h;

  /* 描画範囲の真ん中から描画する */
  h = (int)(GRAPH_HEIGHT / 2) - (int)point2;
  if(h < 0) h = 0;
  graphArray2[counter] = h;  

  counter++;  
}

/* 配列に最後に追加された点を基準に直線を大量に描画します。きちんと動いていればグラフが右から流れてくる形式で表示される */
void drawGraph()
{ /* 独立したstatic変数でカウントアップしているので、必ずsetGraphと同じ回数呼び出さないと描画が狂う。変数を共通にして直した方がいいけどめんどくさい */
  static int counter = 0;
  if (counter == GRAPH_WIDTH) 
    counter = 0;

  /* 画面初期化しないと線が溜まって真っ白になる */
  M5.Lcd.fillScreen(BLACK);  
  for(int i = 0; i < GRAPH_WIDTH; i++)
  {
    /* 現在位置より先と前でインデックスの読み出し方を変える */
    int j = (counter + i < GRAPH_WIDTH) ? i + counter : i + counter - GRAPH_WIDTH;
    
    if(j + 1 == GRAPH_WIDTH)
    { /* 配列で循環させてるので配列末端の場合分けが必要 */
      M5.Lcd.drawLine(K*i, graphArray[j], K*(i+1), graphArray[0], WHITE);
      M5.Lcd.drawLine(K*i, graphArray2[j], K*(i+1), graphArray2[0], GREEN);
    }
    else
    { /* こっちは末端以外 */
      M5.Lcd.drawLine(K*i, graphArray[j], K*(i+1), graphArray[j+1], WHITE);      
      M5.Lcd.drawLine(K*i, graphArray2[j], K*(i+1), graphArray2[j+1], GREEN);
    }
  }
  /* ヒステリシスコンパレータの閾値を表示、この線をまたぐと認識するというわけ */
  M5.Lcd.drawLine(0, GRAPH_HEIGHT-80, K*GRAPH_WIDTH, GRAPH_HEIGHT-80, RED);
  M5.Lcd.drawLine(0, GRAPH_HEIGHT-20, K*GRAPH_WIDTH, GRAPH_HEIGHT-20, BLUE);   
  counter++;  
}

/* 加速度ベクトルの大きさを取得します。正確な値である意味は今のところないので平方根はとってない */
double getNorm(double x, double y, double z)
{
  return (x*x)+(y*y)+(z*z);
}
