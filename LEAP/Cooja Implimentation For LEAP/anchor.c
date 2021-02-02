#include "contiki.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "net/rime.h"
#include <stdio.h>
#include "node-id.h"
#include "dev/leds.h"
#include "node-id.h"
#include <math.h>
#include "dev/cc2420.h"
#include "dev/cc2420_const.h"


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
} database;

struct database currently_information;
struct database *received_data_mote;
struct database routing_table[anchors_num-1];
int counter=0;

struct exploit {
uint8_t id;
float x;
float y;
} exploit;
struct exploit *received_exploit;

/*********************************************************************************************************/
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

/*****************************************************************************************************************/
static struct broadcast_conn broadcast;
static struct unicast_conn unicast;
/*---------------------------------------------------------------------------*/
/* We first declare our processes. */
PROCESS(broadcast_process, "Broadcast process");
PROCESS(flooding_process, "Flooding Process");
PROCESS(unicast_process, "Unicast process");
PROCESS(blink_process, "LED blink process");
PROCESS(display_process, "Display process");
/* The AUTOSTART_PROCESSES() definition specifices what processes to
   start when this module is loaded. We put both our processes
   there. */
AUTOSTART_PROCESSES(&broadcast_process, &unicast_process);
/*---------------------------------------------------------------------------*/
/* This function is called whenever a broadcast message is received. */
static void broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
received_data_mote = packetbuf_dataptr();
process_start(&blink_process, NULL);

/*  Registration of database information in the databases table */
if(received_data_mote->type==0){ 
int i=0; 

for(i=0;i<counter;i++){
if ((received_data_mote->id==routing_table[i].id) &&(received_data_mote->id!=0)){
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

if((i==counter)&&(received_data_mote->id!=node_id)&&(received_data_mote->id!=0)){
routing_table[counter]=*received_data_mote;
received_data_mote->dk +=gethp((float)received_data_mote->tck);
routing_table[counter].dk=received_data_mote->dk;
counter++;
if(received_data_mote->id==1){
rimeaddr_copy(&next_hop, from);
nbr_hop=received_data_mote->dk;
} 
process_start(&flooding_process, NULL);i=0; 
}
}
}
/**************************************************************************************************/
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static void recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
   
received_exploit= packetbuf_dataptr();
if(node_id==1){
int dec_x = received_exploit->x; float frac_x = received_exploit->x - dec_x;  
int dec_y = received_exploit->y; float frac_y = received_exploit->y - dec_y;
printf("Node %u: x=%d.%02d, y=%d.%02d\n",received_exploit->id,dec_x,abs((int)(frac_x*100)),dec_y,abs((int)(frac_y*100)));
}
else{
packetbuf_copyfrom(received_exploit, sizeof(struct exploit));
unicast_send(&unicast, &next_hop);
}
}
static const struct unicast_callbacks unicast_callbacks = {recv_uc};
/*--------------------------unicast_process-----------------------------------------------*/
PROCESS_THREAD(unicast_process, ev, data)
{
PROCESS_EXITHANDLER(unicast_close(&unicast);)   
PROCESS_BEGIN();
unicast_open(&unicast, 146, &unicast_callbacks);
PROCESS_END();
}
/*---------------------------------broadcast_process------------------------------------------*/
PROCESS_THREAD(broadcast_process, ev, data)
{
node_id_burn(1);
cc2420_set_txpower(2);
static struct etimer et1;
currently_information.id=node_id;
currently_information.dk=0;
currently_information.type=0;
currently_information.tck=10;
PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
PROCESS_BEGIN();
broadcast_open(&broadcast, 129, &broadcast_call);
switch(node_id){
case 1: currently_information.x =1; currently_information.y =1;break;
case 2: currently_information.x =7; currently_information.y =6;break;
case 3: currently_information.x =14; currently_information.y =1;break;
}

if (node_id==1){
leds_on(LEDS_YELLOW);
etimer_set(&et1, (CLOCK_SECOND)*2+random_rand() % (CLOCK_SECOND)*2);
PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));
printf("anchor  %u start broadcast\n",node_id);
packetbuf_copyfrom(&currently_information, sizeof(struct database));
broadcast_send(&broadcast);
leds_off(LEDS_YELLOW);
PROCESS_EXIT();
}
else if (node_id==2){
PROCESS_WAIT_EVENT_UNTIL(received_data_mote->id==1);
leds_on(LEDS_YELLOW);
etimer_set(&et1, (CLOCK_SECOND)*10+random_rand() % (CLOCK_SECOND)*2);
PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));
printf("anchor %u start broadcast\n",node_id);
packetbuf_copyfrom(&currently_information, sizeof(struct database));
broadcast_send(&broadcast);
leds_off(LEDS_YELLOW);
PROCESS_EXIT();
}
else if (node_id==3){
PROCESS_WAIT_EVENT_UNTIL(received_data_mote->id==2);
leds_on(LEDS_YELLOW);
etimer_set(&et1, (CLOCK_SECOND)*10+random_rand() % (CLOCK_SECOND)*2);
PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));
printf("anchor %u start broadcast\n",node_id);
packetbuf_copyfrom(&currently_information, sizeof(struct database));
broadcast_send(&broadcast);
leds_off(LEDS_YELLOW);
PROCESS_EXIT();
}
PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(flooding_process, ev, data)
{
static struct etimer et1;
PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
PROCESS_BEGIN();
broadcast_open(&broadcast, 129, &broadcast_call);
if (received_data_mote->type==0){
	process_start(&display_process, NULL);
etimer_set(&et1, (CLOCK_SECOND)+random_rand() % (CLOCK_SECOND));
PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));
received_data_mote->tck=currently_information.tck;
packetbuf_copyfrom(*(&received_data_mote), sizeof(struct database));
broadcast_send(&broadcast);  
}
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
printf("mote=%u,dk=%u/",routing_table[j].id,routing_table[j].dk);
}
printf("\n");
PROCESS_END();
}
/*---------------------------------------------------------------------------*/
