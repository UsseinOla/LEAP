

#include "contiki.h"

#include <stdio.h> /* For printf() */
#include <stdio.h>
#include <math.h>





int dec_x; float frac_x;

/********************************************/
float calcuh(float x){
             int acc=25;
    if(x<0){
        x=(-1)*x;
    float ans=1;
    float temp=1;
    int i;
    for(i=1;i<=acc;i++){
        temp=(temp*x)/i;
        ans=ans + temp;
    }   
    return 1/ans;
       }
    
else{
    float ans=1;
    float temp=1;
    int i;
    for(i=1;i<=acc;i++){
        temp=(temp*x)/i;
        ans=ans + temp;
    }   
    return ans;
       }
}

/******************************************/



double FX(double x,double tck,double density){
    double function=0;
 function=pow(x,2)*calcuh( (-density*M_PI*(pow(tck,2)-pow(x,2)) )/3 );
  
 return function;
}
double solve_integral(double initial, double tck,double density,double cuts){
    double  sumation=0;int i=0;
    double delta, xi, value_i;
 delta=((tck-initial)/cuts);
 for(i=0;i<=cuts;i++){
 xi=(initial+delta*i); 
 value_i=delta*(FX(xi,tck,density));
 sumation=sumation+(value_i);
}  
return sumation;
}
/***********************************************************/
float gethp(){
double cuts;
double initial=0;
double tck=10;
double density=0.08;
double finalanswer=0;
cuts=150;
 finalanswer=(sqrt(3)*density)*solve_integral(initial,tck,density,cuts);

dec_x =finalanswer;  frac_x = finalanswer - dec_x; 
printf("finalanswer  %d.%04d \n ",dec_x,abs((int)(frac_x*10000)));

  return finalanswer;
}
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();

  printf("Hello, EHP \n");

double get=gethp();

  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
