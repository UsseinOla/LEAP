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
char buffer[80];
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
uint8_t  hop_count;
float Av_Hop_Size;
}database; 

struct database *received_data_mote;
struct database routing_table[anchors_num];
int counter_routing_one=0;

struct average_hop_size {
uint8_t type;
uint8_t  id;
float Av_Hop_Size;
}average_hop_size;
struct average_hop_size *received_avh_data;



struct location {
uint8_t id;
float x;
float y;
} location;
struct location *received_location, estimated_received_location;

int ids_counter=0;

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
for(i=0;i<counter_routing_one;i++){
if (received_data_mote->id==routing_table[i].id){
if(received_data_mote->hop_count <routing_table[i].hop_count){
routing_table[i].hop_count=received_data_mote->hop_count; 
if(received_data_mote->id==1){
rimeaddr_copy(&next_hop, from);
nbr_hop=received_data_mote->hop_count;
}
process_start(&flooding_process, NULL);             
}
else{
received_data_mote->hop_count=routing_table[i].hop_count;     
}
break;              
}
} 
if(i==counter_routing_one){
routing_table[counter_routing_one]=*received_data_mote; counter_routing_one++;
if(received_data_mote->id==1){
rimeaddr_copy(&next_hop, from);
nbr_hop=received_data_mote->hop_count;    }
process_start(&display_process, NULL);
process_start(&flooding_process, NULL);i=0;
}
}
else if(received_data_mote->type==1){
received_avh_data= packetbuf_dataptr();
int i;
for(i=0;i<counter_routing_one;i++){
if ((received_avh_data->id==routing_table[i].id)&&(routing_table[i].Av_Hop_Size==0)){
routing_table[i].Av_Hop_Size=received_avh_data->Av_Hop_Size; ids_counter++;
process_start(&display_process, NULL);
process_start(&flooding_process, NULL);
break;
}
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
node_id_burn(12);
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
received_data_mote->hop_count++;
}
else if(received_data_mote->type==1){
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
  *(*(B+i)+0)= pow(routing_table[i].Av_Hop_Size*routing_table[i].hop_count,2)-pow(routing_table[anchors_num-1].Av_Hop_Size*routing_table[anchors_num-1].hop_count,2)-
 pow(routing_table[i].x,2)+pow(routing_table[anchors_num-1].x,2)-pow(routing_table[i].y,2)+pow(routing_table[anchors_num-1].y,2);
  }

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
for(j=0;j<counter_routing_one;j++) {
int dec = routing_table[j].Av_Hop_Size;
float frac = routing_table[j].Av_Hop_Size - dec;
printf("mote=%u,Nbr_hop=%u,Av_Hop_Size=%d.%02d/",routing_table[j].id,routing_table[j].hop_count,dec,abs((int)(frac * 100)));
}
printf("\n");
PROCESS_END();
}
/*---------------------------------------------------------------------------*/