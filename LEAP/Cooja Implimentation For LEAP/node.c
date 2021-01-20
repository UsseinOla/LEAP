#include "contiki.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "net/rime.h"
#include <stdio.h>
#include "dev/leds.h"
#include "node-id.h"
#include "functions.h"
#include <math.h>
#include "dev/cc2420.h"
#include "dev/cc2420_const.h"
/****************************/
#include <stdlib.h>
#include <string.h> //for free() functions
/****************************/
#define anchors_num 3
#define MAX_NEIGHBORS 16

/* This structure holds information about neighbors. */
struct neighbor {
struct neighbor *next;
rimeaddr_t addr;
};
rimeaddr_t next_hop; uint8_t nbr_hop;
/* This structure holds information about database. */
struct database {
uint8_t type;
uint8_t  id;
float  x;
float  y;
uint8_t  dk;
uint8_t tck;
}database; 

struct database *received_data_mote;
struct database routing_table[anchors_num];



struct location {
uint8_t id;
float x;
float y;
} location;
struct location *received_location, estimated_received_location;

int counter=0;
int ids_counter=0;


/************************************************************************************/
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

/**************************************************************************************/

float FX(float x,float tck,float density){
float function=0;
function=pow(x,2)*calcuh( (-density*M_PI*(pow(tck,2)-pow(x,2)) )/3 );
return function;
}
float solve_integral(float initial, float tck,float density,float cuts){
float  sumation=0;int i=0;
float delta, xi, value_i;
 delta=((tck-initial)/cuts);
 for(i=0;i<=cuts;i++){
 xi=(initial+delta*i); 
 value_i=delta*(FX(xi,tck,density));
 sumation=sumation+(value_i);
}  
return sumation;
}
/***********************************************************/
float gethp(float tck){
float cuts;
float initial=0;
float density=0.08;
float finalanswer=0;
cuts=100;
finalanswer=(sqrt(3)*density)*solve_integral(initial,tck,density,cuts);
return finalanswer;
}
/*---------------------------------------------------------------------------*/

static struct broadcast_conn broadcast;
static struct unicast_conn unicast;

/*---------------------------------------------------------------------------*/
/* We first declare our two processes. */
PROCESS(broadcast_process, "Broadcast process");
PROCESS(flooding_process, "Flooding Process");
PROCESS(unicast_process, "Unicast process");
PROCESS(blink_process, "LED blink process");
PROCESS(display_process, "Display process");
PROCESS(trilateral_process, "trilateral process");

/* The AUTOSTART_PROCESSES() definition specifices what processes to
   start when this module is loaded. We put both our processes
   there. */
AUTOSTART_PROCESSES(&broadcast_process, &unicast_process);
/*---------------------------------------------------------------------------*/
/* This function is called whenever a broadcast message is received. */
static void broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{ 
received_data_mote = packetbuf_dataptr();
/*  Registration of database information in the databases table */ 
if(received_data_mote->type==0){
int i=0; 
for(i=0;i<counter;i++){
if (received_data_mote->id==routing_table[i].id){
if(received_data_mote->dk <routing_table[i].dk){
routing_table[i].dk=received_data_mote->dk; 
if(received_data_mote->id==1){
rimeaddr_copy(&next_hop, from);
nbr_hop=received_data_mote->dk;
}
process_start(&flooding_process, NULL);             
}
else{
received_data_mote->dk=routing_table[i].dk;     
}
break;              
}
} 
if(i==counter){
routing_table[counter]=*received_data_mote;
received_data_mote->dk +=gethp((float)received_data_mote->tck);
routing_table[counter].dk=received_data_mote->dk;
routing_table[counter].tck=10;
counter++;
ids_counter++;

if(received_data_mote->id==1){
rimeaddr_copy(&next_hop, from);
nbr_hop=received_data_mote->dk;    }
//process_start(&display_process, NULL);
process_start(&flooding_process, NULL);i=0;
}

}

/*************************************************************/
}

/* This is where we define what function to be called when a broadcast
   is received. We pass a pointer to this structure in the
   broadcast_open() call below. */
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
/*---------------------------------------------------------------------------*/
/* This function is called for every incoming unicast packet. */
static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
received_location= packetbuf_dataptr();
packetbuf_copyfrom(received_location, sizeof(struct location));
unicast_send(&unicast, &next_hop);
}
static const struct unicast_callbacks unicast_callbacks = {recv_uc};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(broadcast_process, ev, data)
{
node_id_burn(1);
cc2420_set_txpower(2);
PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
PROCESS_BEGIN();
broadcast_open(&broadcast, 129, &broadcast_call);
PROCESS_END();
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(flooding_process, ev, data)
{
static struct etimer et1;
PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
PROCESS_BEGIN();
broadcast_open(&broadcast, 129, &broadcast_call);
if(received_data_mote->type==0){
received_data_mote->dk++;
if(ids_counter==3){
etimer_set(&et1, (CLOCK_SECOND)*2+random_rand() % (CLOCK_SECOND)*2);
PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));   
process_start(&trilateral_process, NULL);
}
}
PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(unicast_process, ev, data)
{
PROCESS_EXITHANDLER(unicast_close(&unicast);)   
PROCESS_BEGIN();
unicast_open(&unicast, 146, &unicast_callbacks);
PROCESS_END();
}
/*-----------------------------------------------------------------------------------*/
 PROCESS_THREAD(trilateral_process, ev, data)
{
static struct etimer et1;
PROCESS_EXITHANDLER(unicast_close(&unicast);)
PROCESS_BEGIN();
unicast_open(&unicast, 146, &unicast_callbacks);
/****************Calcule Matrix A:(anchors_num-1*2)****************/
/****************Calcule Matrix A:(anchors_num-1*2)****************/

 int l=0,l1=0;
int dec_x; float frac_x;

 int i,j; float **A;
A=(float**)malloc((anchors_num-1)*sizeof(float*));
  for (i=0;i<anchors_num-1;i++)
     A[i]=(float*)malloc(3*sizeof(float));

  for(i=0;i<anchors_num-1;i++)
     for(j=0;j<2;j++)
  if (j==0)
    *(*(A+i)+j)=-2*(routing_table[i].x-routing_table[anchors_num-1].x);
  else if (j==1)
    *(*(A+i)+j)=-2*(routing_table[i].y-routing_table[anchors_num-1].y);


/****************Calcule Matrix B:(nbr_received_data_mote-1*1)*****************/
   float **B;float eval=0;
  B=(float**)malloc((anchors_num-1)*sizeof(float*));
  for (i=0;i<anchors_num-1;i++)
    B[i]=(float*)malloc(1*sizeof(float));
  
  for(i=0;i<anchors_num-1;i++)
  {
    /*dec_x =routing_table[i].dk;  frac_x = routing_table[i].dk - dec_x; 
printf("routing_table[i].dk=%d.%04d \n",dec_x,abs((int)(frac_x*10000)));*/

  *(*(B+i)+0)= pow(routing_table[i].dk,2)-pow(routing_table[anchors_num-1].dk,2)-
 pow(routing_table[i].x,2)+pow(routing_table[anchors_num-1].x,2)-pow(routing_table[i].y,2)+pow(routing_table[anchors_num-1].y,2);
  }

/*
printf("B\n");
  for(l=0;l<2;l++){
    for(l1=0;l1<1;l1++){
dec_x =B[l][l1];  frac_x = B[l][l1] - dec_x; 
printf("%d.%04d ",dec_x,abs((int)(frac_x*10000)));
}
printf("\n");
  }*/



 /****************Calcule Transpose A: A_T(2*nbr_anc-1)*****************/
 float **A_T;
 A_T=matrix_transpose(A,anchors_num-1,2);

 /****************Calcule A_T*A:ATA(2*2)*****************/
 float **ATA;
 ATA= matrix_multiplication(A_T,2,anchors_num-1,A,2);
  
 /****************Calcule ATA inverse:ATA_Inv(2*2)*****************/
 float **ATA_Inv;
 ATA_Inv=matrix_inverse(ATA,2);

 /****************Calcule ATA_Inv*A_T: ATA_AT(2*nbr_anc-1)*****************/
 float **ATA_AT;
 ATA_AT=matrix_multiplication(ATA_Inv, 2, 2, A_T, anchors_num-1);

 /****************Calcule ATA_AT*B: P(2*1)*****************/
 float **P;        
 P=matrix_multiplication(ATA_AT, 2, anchors_num-1, B, 1);

estimated_received_location.id=node_id;
estimated_received_location.x=*(*(P+0)+0); estimated_received_location.y=*(*(P+1)+0);
leds_on(LEDS_GREEN);
etimer_set(&et1,(CLOCK_SECOND)*(nbr_hop)+random_rand() % (CLOCK_SECOND)*nbr_hop);
PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));
leds_off(LEDS_GREEN);
packetbuf_copyfrom(&estimated_received_location, sizeof(struct location));
unicast_send(&unicast, &next_hop);




for(i=0;i<anchors_num;i++)
free(A[i]);
free(A);


for(i=0;i<anchors_num;i++)
free(B[i]);
free(B);

free(A_T);
free(ATA);
free(ATA_Inv);
free(ATA_AT);
free(P);
PROCESS_EXIT();
PROCESS_END();
 }
/*---------------------------------------------------------------------------*/
 PROCESS_THREAD(blink_process, ev, data)
 {
 static struct etimer timer;
 PROCESS_BEGIN(); 
 leds_on(LEDS_GREEN);
 etimer_set(&timer, CLOCK_SECOND/4);
 PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
 leds_off(LEDS_GREEN);     
 PROCESS_END();
 }
/*---------------------------------------------------------------------------*/
 PROCESS_THREAD(display_process, ev, data)
{
PROCESS_BEGIN(); 
int j=0;
for(j=0;j<counter;j++) {
printf("anchor=%u,hops=%u",routing_table[j].id,routing_table[j].dk);
}
printf("\n");
PROCESS_END();
}
/*---------------------------------------------------------------------------*/