#include <opencv2/videoio.hpp> 
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <opencv2/imgcodecs.hpp> 
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <wiringPi.h>
#include <unistd.h>
#include <opencv2/features2d.hpp>
#include <opencv2/objdetect.hpp>

void izq(); // movimineto del servo a la izquierda
void der(); //movimiento del servo a la derecha 
void inter();// pendientdente
int LED = 26; //interrupcion para clasificador del servo 
int LED2= 28; //motor
int LED3= 29; //solenoide 
int LED4 =27; //interrupcion del solenoide
int contador; //vairables para el uso del solenoide (ineterrupcion) 
int contador2;	
//pwm export pins //servo    clasificar  las galletas
void export_pwm(int pwm);
void set_pwm_period(int pwm, int period);
void set_pwm_duty_cycle(int pwm, int duty_cycle);
void enable_pwm(int pwm, int enable);
void control_servo(int pwm, int position);
void set();
void galletas();// avvienta galletas 
// variables de funcionamientos 
int pwm = 0;  // Canal PWM 0
int period = 20000000;  // 20 ms
int duty_cycle = 1500000;  // 1.5 ms
unsigned int microsecond = 1000000;
// Function for Face Detection

using namespace std;
using namespace cv;
void blink();//DESARTAR
Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
///FUNCION MEDICION DE OBJETO
void thresh_callback(int, void* ); //funcion de area 
void detectAndDraw( Mat& img, CascadeClassifier& cascade,CascadeClassifier& nestedCascade, double scale );//funcion del cascade 
string cascadeName, nestedCascadeName;

int main()
{      //seteo de GPIO
       wiringPiSetup();
       pinMode(LED,INPUT); 
       pinMode(LED2,OUTPUT);
       pinMode(LED3,OUTPUT);
       pinMode(LED4,INPUT); 
       VideoCapture cap(0);
         cap.set(CAP_PROP_FRAME_WIDTH, 320);//TAMNO DE VENTANA PARA NO RELENTIZAR LA DETECCION 
         cap.set(CAP_PROP_FRAME_HEIGHT, 240);
          
    if (!cap.isOpened()) {
        cerr << "Error al abrir la ca mara" << endl;
        return -1;
    }
    Mat frame;
    Mat image;
    CascadeClassifier cascade, nestedCascade; 
    double scale=1;
    nestedCascade.load( "cascade.xml");// ENTRENAMIMIENOT DEL CASCADE 
    cascade.load( "cascade.xml" ) ;  // ENTRENAMOIENTO DEL CASCADE 
    while (true) {
     // cap.set(CAP_PROP_FRAME_WIDTH, 320);//TAMNO DE VENTANA PAR>
     // cap.set(CAP_PROP_FRAME_HEIGHT, 240);
      cap>>frame;
       cap.read(frame);
        if (frame.empty()) {
            cerr << "Fin del video" << endl;
            break;
          }
     Mat frame1 = frame.clone();
     detectAndDraw( frame1, cascade, nestedCascade, scale ); 

//FILTRO BINARIO (USO OPCIONAL PARA CALIBRAR LA CAMARA)                 
        Mat bw;
        cvtColor(frame, bw, COLOR_BGR2GRAY);
        threshold(bw, bw, 40, 255, THRESH_BINARY | THRESH_OTSU);
        cvtColor( frame, src_gray,COLOR_BGR2GRAY );
        thresh_callback( 0, 0 );//refrescar funcion de area 
	digitalWrite(LED2,LOW);//activar la banda 
	delay(30);
	digitalWrite(LED2,HIGH);//activar la banda 
	delay(30);
       // Motor();

//fitro  binario.
           for (int y = 0; y < bw.rows; y++) {
           for (int x = 0; x < bw.cols; x++) {
           if (bw.at<uchar>(y, x) > 0) {
            // Conserva el color original
        } 
           
    }
}

      //  imshow("FILTRO BINARIO", bw);   
        //imshow("Original", frame);

        if (waitKey(100) == 113) { // Presiona q para salir
            digitalWrite(LED2,HIGH);         //apaga la banda 
        //        OFF();
	       	break;
        }
   }

    return 0;



}

//funcion de deteccion de objetos y su tama√o 
/** @function thresh_callback */
void thresh_callback(int, void* )
{
 export_pwm(pwm);
       usleep(1);  // Espera para que el sistema exporte el PWM  exporto el pwm 
       set_pwm_period(pwm, period);
       set_pwm_duty_cycle(pwm, duty_cycle);
       enable_pwm(pwm, 1);


  int resultado=0;// el area en pixeles 
  int pins = digitalRead(LED);//leer el pin de interrupcion 
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  Mat blurred, edges;  //filtado del area 
 
   GaussianBlur(src_gray, blurred, Size(5, 5), 0); //filtrado de el area (permite tener un filtatrado de imagen )  
  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY  | THRESH_OTSU );//funcion para el treshold del area o controo
  findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_TC89_L1, Point(0, 0) );
  /// Find the rotated rectangles and ellipses for each contour
  vector<RotatedRect> minRect( contours.size() );
  vector<RotatedRect> minEllipse( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
     { minRect[i] = minAreaRect( Mat(contours[i]) );
       if( contours[i].size() > 5 )
         { minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
     }

  /// Draw contours + rotated rects + ellipses
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
 if (!contours.empty()){
 // Procesa el primer contorno encontrado para el sistema.
        RotatedRect minRect = minAreaRect(Mat(contours[0]));
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        Point2f rect_points[4];
        minRect.points(rect_points);
        for (int j = 0; j < 4; j++) {
            line(drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8);
             Rect bounding_rect = boundingRect(contours[0]);
             int area = bounding_rect.width * bounding_rect.height;// multiplica la cantidad de pixeles 
             cout <<" (Pixeles) Area : " << area << endl;//imprime el area
//             delay(10);
             resultado=area;
}
}
//funcion de interrupcion y clasificacion 
	if(resultado>=15000 && resultado <=20000 ){
	der();
        cout<<"giro a la derecha "<<endl;
}
if(resultado>=22000 && resultado <=28000 ){
	izq();
        cout<<"giro a la izquierda "<<endl;
}
   findContours( src, contours, hierarchy,
 RETR_CCOMP, CHAIN_APPROX_SIMPLE );
 
 // iterate through all the top-level contours,
 // draw each connected component with its own random color
 int idx = 0;
 for( ; idx >= 0; idx = hierarchy[idx][0] )
 {
 Scalar color( rand()&255, rand()&255, rand()&255 );
drawContours( drawing, contours, idx, color, FILLED, 8, hierarchy );
 }

// namedWindow( "Binary filter", 1 );
// imshow( "Components", drawing );
  /// Show in a window
  namedWindow( "Contours",WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
}


//No hacer caso a esta funciion solo era para  una prueba
void blink(){
                digitalWrite(LED,HIGH);         
                delay(100);
                digitalWrite(LED, LOW);    
                delay(100);

}

//DECLARACION DEL GPIO PARA EL PWM  Y FUNCIONALIDAD DE ESTA 
//exportacion del pwm 
void export_pwm(int pwm) {
    FILE *fp = fopen("/sys/class/pwm/pwmchip0/export", "w");
    if (fp == NULL) {
        perror("Error al exportar PWM");
        exit(EXIT_FAILURE);
    }
    fprintf(fp, "%d", pwm);
    fclose(fp);
}

// Funcion para configurar el periodo del PWM
void set_pwm_period(int pwm, int period) {
    char path[50];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip0/pwm%d/period", pwm);

    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        perror("Error al configurar el periodo del PWM");
        exit(EXIT_FAILURE);
    }
    fprintf(fp, "%d", period);
    fclose(fp);
}

// Funcion para configurar el ciclo de trabajo del PWM
void set_pwm_duty_cycle(int pwm, int duty_cycle) {
    char path[50];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", pwm);

    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        perror("Error al configurar el ciclo de trabajo del PWM");
        exit(EXIT_FAILURE);
    }
    fprintf(fp, "%d", duty_cycle);
    fclose(fp);
}
// Funcion para habilitar o deshabilitar el PWM
void enable_pwm(int pwm, int enable) {
    char path[50];
 snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip0/pwm%d/enable", pwm);

    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        perror("Error al habilitar/deshabilitar el PWM");
        exit(EXIT_FAILURE);
    }
    fprintf(fp, "%d", enable);
    fclose(fp);
}


// Funcion para controlar el rango del servomotor servomotor
void control_servo(int pwm, int position) {
    int min_duty_cycle = 1000000;  // 1 ms
    int max_duty_cycle = 2000000;  // 2 ms
    int duty_cycle = min_duty_cycle + position * (max_duty_cycle - min_duty_cycle) / 180;
    set_pwm_duty_cycle(pwm, duty_cycle);
}

void izq(){
       	int pos;
   //for( pos;pos<=190;pos+=30){ 
       control_servo(pwm,145);
//}
}
void der(){
       	int pos;
  //for (pos;pos>=20;pos-=30){
 control_servo(pwm,35);
//}

}




 //FUNCION PARA LA DETECCION DE GALLETAS ROTAS CASCADES
void detectAndDraw( Mat& img, CascadeClassifier& cascade,CascadeClassifier& nestedCascade,double scale)
{
    int pins2=digitalRead(LED4); //DECLARAMOS EL PIN QUE ESTAREMOS LEYENDO PARA EL CLASIFICADOR DE GALLETAS ROTAS
    vector<Rect> faces, faces2;  //DECLARAMOS UN VECTOR PARA PODER DETECTAR CONTORNOS
    Mat gray, smallImg;
 
    cvtColor( img, gray, COLOR_BGR2GRAY ); // Convert to Gray Scale
    double fx = 1 / scale;
 
    // Resize the Grayscale Image 
    resize( gray, smallImg, Size(), fx, fx, INTER_LINEAR );  
equalizeHist( smallImg, smallImg );
 
    // Detect faces of different sizes using cascade classifier 
    cascade.detectMultiScale( smallImg, faces, 1.1, 
                            2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
       
    //DIBUJO ALREDREOR DE LAS GALLETAS QUE TIENEN DA—O 
    for ( size_t i = 0; i < faces.size(); i++ )
    {
  
	 cout<<"contador : "<<contador<<endl;
        
	 Rect r = faces[i];
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;
        Scalar color = Scalar(255, 0, 0); // Color for Drawing tool
        int radius;
        double aspect_ratio = (double)r.width/r.height;
       	if (0.75< aspect_ratio && aspect_ratio <1.3){       //si se cumple el rango del circulo dibujalo    
            center.x = cvRound((r.x + r.width*0.5)*scale);
            center.y = cvRound((r.y + r.height*0.5)*scale);
            radius = cvRound((r.width + r.height)*0.25*scale);
            circle( img, center, radius, color, 3, 8, 0 );
          //  contador=faces.size();
           contador++;//haz una suma del a deteccion(memoria para cuando se detecta daÒo)

	}
     
    }
    contador+=contador2; // pasamos el valor del contador para poder externarlo fuera del for

while(pins2!=LOW) {     //activa el piston hasta que el sensor infrarrojo sea difernete de 0 (activado) 
   // cout<<"contadord"<<contador2<<endl;	
    if(contador>0){
     der();
     galletas();
     cout<<"PISTO ACTIVADO"<<endl;
    // contador2=0;
    }
    contador=0;
    break;
  }
   imshow( "COOKIE DETECTOR", img ); 
   

}
void galletas(){ // funcion para el funcionaminento  del sensor 
                digitalWrite(LED3,HIGH);         /* write high on GPIO */
                delay(100);
                digitalWrite(LED3, LOW);         /* write low on GPIO */
                delay(100);

   
}


