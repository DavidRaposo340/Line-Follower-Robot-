#include <avr/io.h>
#include <avr/interrupt.h>
#include "serial_printf.h"
#include <avr/eeprom.h>

#define T1BOTTOM 65536-31250   //Note: Mode 0 
//queremos 125ms => (125ms*16Mhz)/prescaler(64)=31250


uint8_t state_FSM0, next_state_FSM0, state_FSM1, next_state_FSM1, state_FSM2, next_state_FSM2, state_FSM3, next_state_FSM3, state_FSM10, next_state_FSM10;
uint8_t sw_1, sw_prev_1;


volatile uint16_t T1, Tshow;  //timers
volatile uint8_t dutycycle_direita, dutycycle_esquerda;  //pwm
volatile uint8_t BT;  //valor vindo do bluetooth

volatile uint16_t sensor_IR_1, sensor_IR_2, sensor_IR_3, sensor_IR_4, sensor_IR_5;  //vetor dos sensores
volatile uint8_t canal;  //posiçao do vetor dos sensores




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Timers
ISR(TIMER1_OVF_vect)
{
  TCNT1 = T1BOTTOM; //reload TC1

  if(Tshow) Tshow--;
  if (serial_receive_ready()) {       // Test if there is serial data to be read
       BT = serial_receive();   // Read serial data
  }
}

void tc1_init(void) {
  cli();
  TCCR1B = 0;          // Stop TC1
  TIFR1 = (7<<TOV1)    // Clear pending intr
        | (1<<ICF1);
  TCCR1A = 0;          // Mode zero
  TCNT1 = T1BOTTOM;    // Load BOTTOM value
  TIMSK1 = (1<<TOIE1); // Enable Ovf intrpt
  TCCR1B &= ~(1 << CS12);   // Start TC1 (TP=64) ->>> prescaler64= 011 = 3  -> tambem se poderia escrever TCCR1B = 3; 
  TCCR1B |= (1 << CS11);
  TCCR1B |= (1 << CS10);
  sei();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNCOES LE sensores infravermelhos ->> 1 = detetou preto

uint8_t stop_frente(void)
{
	return !(PIND &(1<<7));
}

uint8_t max_esq (void)
{
  if(sensor_IR_1<=500)
    return 1;
  else
    return 0;
}
uint8_t esq (void)
{
  if(sensor_IR_2<=500)
    return 1;
  else
    return 0;
}
uint8_t meio (void)
{
  if(sensor_IR_3<=500)
    return 1;
  else
    return 0;
}
uint8_t dir (void)
{
  if(sensor_IR_4<=500)
    return 1;
  else
    return 0;
}
uint8_t max_dir (void)
{
  if(sensor_IR_5<=500)
    return 1;
  else
    return 0;
}

void read_adc(unsigned char chan) { //funçao fornecida nos slides
 // escolher o canal...
 ADMUX = (ADMUX & 0xF0) | (chan & 0x0F);
 // iniciar a conversão
 // em modo manual (ADATE=0)
 ADCSRA |= (1<<ADSC);
 // esperar pelo fim da conversão

}

ISR(ADC_vect)
{
  //tive de dar um shift.. mas assim esta correto! 
  if (canal==0)
  {
    read_adc(1);   
    sensor_IR_5=ADC;//max_dir()
  }
  else if (canal==1)
  {
    read_adc(2);
    sensor_IR_1=ADC;//max_esq()
  }
  else if (canal==2)
  {
    read_adc(3); 
    sensor_IR_2=ADC;//esq()
  }
  else if (canal==3)
  {
    read_adc(4); 
    sensor_IR_3=ADC;//meio()
  }
  else if (canal==4)
  {
    read_adc(5); 
    sensor_IR_4=ADC;//dir()
  }
  canal++;
  if(canal>4)
  {
    canal=0;
    ADMUX &= ~(  ( 1 << MUX0 ) | ( 1 << MUX1 ) | ( 1 << MUX2 ) | ( 1 << MUX3 )  );
  }
}


void ADC_init (){ 
  
  cli();
  ADMUX =  ( 1 << REFS0 )  ;
  ADCSRA = ( 1 << ADEN ) | ( 1 << ADIE ) | ( 1 << ADPS0 ) | ( 1 << ADPS1 ) | ( 1 << ADPS2 );
  DIDR0 = ( 1 << ADC1D ) | ( 1 << ADC2D ) | ( 1 << ADC3D ) | ( 1 << ADC4D ) | ( 1 << ADC5D ); //desativar o buffer dos digital pin (nao é obrigatorio.. é segurança)
  ADCSRA |= ( 1 << ADSC );
  sei();

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 void EEPROM_write(unsigned int uiAddress, uint8_t ucData)
  {
      /* Wait for completion of previous write */
      while(EECR & (1<<EEPE))
      ;
      /* Set up address and Data Registers */
        EEAR = uiAddress;
        EEDR = ucData;
      /* Write logical one to EEMPE */
        EECR |= (1<<EEMPE);
      /* Start eeprom write by setting EEPE */
        EECR |= (1<<EEPE);
  }

  uint8_t EEPROM_read(unsigned int uiAddress)
  {
      /* Wait for completion of previous write */
        while(EECR & (1<<EEPE))
      ;
      /* Set up address register */
        EEAR = uiAddress;
      /* Start eeprom read by writing EERE */
      EECR |= (1<<EERE);
      /* Return data from Data Register */
      return EEDR;
  }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(TIMER2_OVF_vect)
{
    OCR2A= (dutycycle_direita/100.0)*255; //converte para valor de 0-255
    OCR2B= (dutycycle_esquerda/100.0)*255; //converte para valor de 0-255
}


void motor_direita(uint8_t velocidade, uint8_t frente )
{ 
  //velocidade valor de 0 a 100
  // frente=1 para motor andar para frente, frente=0 para motor andar para tras
  
  if (frente) //anda para frente
  {
    PORTB |= (1 << 4);  //ain2 on
    PORTB &= ~(1 << 2);  //ain1 off
  }
  else //anda para tras
  {
    PORTB &= ~(1 << 4);  //ain2 off
    PORTB |= (1 << 2);  //ain1 on
  }

  if (velocidade==0)
  {
      PORTB &= ~(1 << 2);  //ain1 off
      PORTB &= ~(1 << 4);  //ain2 off
  }

  dutycycle_direita=velocidade;
  //OCR2A= (velocidade/100.0)*255; //converte para valor de 0-255

  return;
}

void motor_esquerda(uint8_t velocidade, uint8_t frente )
{ 
  //velocidade valor de 0 a 100
  // frente=1 para motor andar para frente, frente=0 para motor andar para tras
  
  if (frente) //anda para frente
  {
    PORTD |= (1 << 4);  //bin2 on
    PORTD &= ~(1 << 2);  //bin1 off
  }
  else //anda para tras
  {
    PORTD &= ~(1 << 4);  //bin2 off
    PORTD |= (1 << 2);  //bin1 on
  }

  if (velocidade==0)
  {
      PORTD &= ~(1 << 2);  //bin1 off
      PORTD &= ~(1 << 4);  //bin2 off
  }

  dutycycle_esquerda=velocidade;  
  //OCR2B= (velocidade/100.0)*255; //converte para valor de 0-255

  return;
}

void frente( uint8_t on){
  if (on){
     motor_esquerda(80, 1);
     motor_direita(80, 1);
  }
  else return;
}

void tras( uint8_t on){
  if (on){
     motor_esquerda(80, 0);
     motor_direita(80, 0);
  }
  else return;
}

void vira_dir( uint8_t on){
  if (on){
     motor_esquerda(85, 1);
     motor_direita(55, 1);
  }
  else return;
}

void vira_esq( uint8_t on){
  if (on){
     motor_esquerda(55, 1);
     motor_direita(85, 1);
  }
  else return;
}

void vira_dir_tras( uint8_t on){
  if (on){
     motor_esquerda(85, 0);
     motor_direita(55, 0);
  }
  else return;
}

void vira_esq_tras( uint8_t on){
  if (on){
     motor_esquerda(55, 0);
     motor_direita(85, 0);
  }
  else return;
}

void vira_forte_dir( uint8_t on){
  if (on){
     motor_esquerda(50, 1);
     motor_direita(50, 0);
  }
  else return;
}

void vira_forte_esq( uint8_t on){
  if (on){
     motor_esquerda(50, 0);
     motor_direita(50, 1);
  }
  else return;
}

void para( uint8_t on){
  if (on){
     motor_esquerda(0, 1);
     motor_direita(0, 1);
  }
  else return;
}


void tc2_init (void){ //pwm
  cli();
  TCCR2A=0; //limpar o que la estava
  TCCR2B=0; //limpar o que la estava
  
  TCCR2A |= (1 << COM2A1)  | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); //pwm nao invertido (tanto OC2A e OC2B); Modo 3
  
  TIMSK2 = (1<<TOIE2); // Enable Ovf intrpt

  OCR2A=0;  // pmw para motor A com duty cycle de 0% (iniciar a off por segurança)
  OCR2B=0;  // pmw para motor B com duty cycle de 0% (iniciar a off por segurança)

  TCCR2B |= (1 << CS22) | (1 << CS21); //pwm Modo 3; F oc2a = 244MHz e F clk = 16MHz e TOP=0xFF (pq estamos em modo3) ->> temos um prescaler(N) de 256 (Foc2a=Fclk/(N*(1+TOP)))
  
  sei();
  // depois para aumentar a velocidade temos de aumentar o ocr2a e/ou ocr2b
  // vai de 0 a 255... mas dutycycle vai de 0 a 100 (OCR2A= (dutycycle/100.0)*255.0 )     
}


///////////////////////////////////////////////////
//PID
 uint8_t ERRO, ERRO_anterior=0, lado;  //ERRO PID / lado=1 vira direita; lado=0 vira esquerda
 uint8_t ki=0 , kp=50 , kd=40;  
 uint8_t I , P , D, PID;
 uint8_t vel_esq, vel_dir;  
 uint8_t vel_E=40, vel_D=40;  

void medir_erros(){ //erros negativos para esquerda, positvos para direita
  uint8_t ME, E, M, D, MD;
  ME=max_esq();
  E=esq();
  M=meio();
  D=dir();
  MD=max_dir();

  if      ( (ME==0) && (E==0) && (M==1) && (D==0) && (MD==0) ){
    ERRO=0;
    lado=1;
  } 
  else if ( (ME==0) && (E==0) && (M==1) && (D==1) && (MD==0) ) {
    ERRO=1;
    lado=1;
  } 
  else if ( (ME==0) && (E==0) && (M==0) && (D==1) && (MD==0) ) {
    ERRO=3;
    lado=1;
  } 
  else if ( (ME==0) && (E==0) && (M==0) && (D==1) && (MD==1) ) {
    ERRO=4;
    lado=1;
  } 
  else if ( (ME==0) && (E==0) && (M==0) && (D==0) && (MD==1) ) {
    ERRO=5;
    lado=1;
  } 
  
  else if ( (ME==0) && (E==1) && (M==1) && (D==0) && (MD==0) ) {
    ERRO=1;
    lado=0;
  } 
  else if ( (ME==0) && (E==1) && (M==0) && (D==0) && (MD==0) ) {
    ERRO=3;
    lado=0;
  } 
  else if ( (ME==1) && (E==1) && (M==0) && (D==0) && (MD==0) ) {
    ERRO=4;
    lado=0;
  } 
  else if ( (ME==1) && (E==0) && (M==0) && (D==0) && (MD==0) ) {
    ERRO=5;
    lado=0;
  } 
  
}

void calcular_PID(){
  
  if (ERRO==0)  I=0;

  P=ERRO;
  I=I+ERRO;
  if (I>100)  I=100;
  else if (I<-100)  I=-100;
  D=ERRO- ERRO_anterior;
  PID=(kp* (P*0.1) )+ (ki* (I*0.1) )+ (kd* (D*0.1) );
  //PID=(kp* (P) )+ (ki* (I) )+ (kd* (D) );
  ERRO_anterior=ERRO; 

}

void controlar_velocidade_PID(){
  if(lado==1){ // vira direita
    vel_esq=vel_E + PID;
    vel_dir=(vel_D-PID);

  }

  else if(lado==0) { // vira equerda 
    vel_esq=(vel_E-PID) ;
    vel_dir=vel_D + PID;

  }

    motor_direita(vel_dir, 1);
    motor_esquerda(vel_esq, 1);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{
  //inicializacao FRENTE PARA
  DDRD &= ~(1 << 7);


  //inicializaçao das saidas pwm
  DDRB |= (1 << 3); //definir pino 11 como output (pwm)
  DDRD |= (1 << 3); //definir pino 3 como output (pwm)

  //inicializaçao das saidas de controlode direçao de motores
  DDRB |= (1 << 2); 
  DDRB |= (1 << 4);
  DDRD |= (1 << 2); 
  DDRD |= (1 << 4); 
   
  if ((EEPROM_read(1<<1) > 101) || (EEPROM_read(1<<1) < 100)) {  //se a eeprom nao tiver o estado da maquina 10 (que vai de 100 a 101), usar o estado 100
    EEPROM_write((1<<1), 100);
  }
 

  
  
  tc1_init();
  tc2_init();
  ADC_init();

  //cli();
  //Tshow=2;
  //sei();

  printf_init();        // Init the serial port to have the ability to printf
  printf("Serial I/O Demo\n");   // into a terminal  
  
  

  while (1) {

//MAQUINA DE ESTADOS 0 - MODOS

    switch (state_FSM0) {
  
      case 0: { //stop
        if ( BT==65 ) {                
  		  	next_state_FSM0 = 1;  //manual
        } 
        else if ( BT==66 ){
  		  	next_state_FSM0 = 2;  //linha
        } 
        else if ( BT==67 ){
  		  	next_state_FSM0 = 3;  //linha
        }
      } break;
      
      case 1: {    
        if (BT==68) {                
  		  	next_state_FSM0 = 0;
        }        
      } break;

      case 2: {    
        if (BT==68) {                
  		  	next_state_FSM0 = 0;
        }        
      } break;

      case 3: {    
        if (BT==68) {                
  		  	next_state_FSM0 = 0;
        }        
      } break;
      
      default:
        next_state_FSM0 = 0;  // in case of error goto state_FSM0 0
      break;
    }
  
    if (next_state_FSM0 != state_FSM0) {   // If needed, change state_FSM0
      state_FSM0 = next_state_FSM0;
    }
  

//MAQUINA DE ESTADOS 1 - LINHA

    switch (state_FSM1) {
  
      case 10: {

        if ( state_FSM0 == 2) {                
  		  	next_state_FSM1 = 11;  
        } 
      } break;
      
      case 11: {
          medir_erros();
          calcular_PID();
          controlar_velocidade_PID();

        if ( state_FSM0 == 0  ) {                
  		  	next_state_FSM1 = 10; 
        } 

        else if ( (max_esq()==1  /*&& meio()==0*/ &&  max_dir()==1 )  /*|| (esq()==1  &&  max_dir()==1) || (max_esq()==1  &&  dir()==1)*/ ) {                       //  bifurcaçao    
  		  	next_state_FSM1 = 14; 
        } 

        else if (  (max_esq()==1 /*&& esq()==1*/ /*&& meio()==1*/ && dir()==0 && max_dir()==0 ) ) {     //  curva 90º esq       
  		  	next_state_FSM1 = 12; 
        } 

        else if ( (max_esq()==0 && esq()==0 /*&& meio()==1*/ /*&& dir()==1*/ && max_dir()==1 ) ) {     //  curva 90º dir  
  		  	next_state_FSM1 = 13; 
        } 
      } break;
      
      case 12: {  //    curva 90º esq
        motor_esquerda(27, 0);
        motor_direita(40, 1);
        
        if (  (max_esq()==1  &&   max_dir()==1 )) {                        //  bifurcaçao 
  		  	next_state_FSM1 = 14; 
        }

        else if (  (max_esq()==0  && (meio()==1 && ( esq()==1 || dir()==1 ) ) &&  max_dir()==0 ) ) {       //  encontrou caminho       
  		  	next_state_FSM1 = 11; 
        }
      } break;
      
      case 13: {  //    curva 90º dir
        motor_esquerda(40, 1);
        motor_direita(27, 0);
        
        if (  (max_esq()==1  &&   max_dir()==1 ) ) {                        //  bifurcaçao 
  		  	next_state_FSM1 = 14; 
        }

        else if (  (max_esq()==0  && (meio()==1 && ( esq()==1 || dir()==1 ) ) &&  max_dir()==0 ) ) {       //  encontrou caminho       
  		  	next_state_FSM1 = 11; 
        }
      } break;

      case 14: {  //    bifurcaçao
        if (state_FSM10 == 100){ //vai pelo caminho da direita
          motor_esquerda(45, 1);
          motor_direita(25, 0);
        }
        else if (state_FSM10 == 101){ //vai pelo caminho da esquerda
          motor_esquerda(25, 0);
          motor_direita(45, 1);
        }

        
        if (  (max_esq()==0  && (meio()==1 && ( esq()==1 || dir()==1 ) ) &&  max_dir()==0 ) ) {       //  encontrou caminho       
  		  	next_state_FSM1 = 11; 
        }
      } break;

      
      default:
        next_state_FSM1 = 10;  // in case of error goto state_FSM1 10
      break;
    }
  
    if (next_state_FSM1 != state_FSM1) {   // If needed, change state_FSM0
      state_FSM1 = next_state_FSM1;
    }

    if(state_FSM1 != 10 && (state_FSM0 != 2 ) ){ //mudou o modo
      state_FSM1=10;
    }


//MAQUINA DE ESTADOS 10 - decisão na bifurcaçao

    switch (state_FSM10) {
  
      case 100: { //dir
        if ( (BT == 4) ) {                
  		  	next_state_FSM10 = 101;  
        } 
      } break;

      case 101: { //esq
        if ( (BT == 2) ) {                
  		  	next_state_FSM10 = 100;  
        } 
      } break;
      
      default:
        next_state_FSM10 = EEPROM_read((1<<1));   // inicialmente, procurar o valor guardado na eeprom
      break;
    }
    if (next_state_FSM10 != state_FSM10) {   // If needed, change state_FSM10
      state_FSM10 = next_state_FSM10;
      EEPROM_write((1<<1) , state_FSM10);    //guardar o valor do estado da maquina 10 na eeprom
    }





//MAQUINA DE ESTADOS 2 - manual

    switch (state_FSM2) {

      case 20: {
        if (state_FSM0 == 1) next_state_FSM2 = 21;
      } break;

      case 21: { //parado 
        if ((BT == 1) ){
          next_state_FSM2 = 22;
        }

        else if ( (BT == 2) ){
          next_state_FSM2 = 24;
        }

        else if ( (BT == 3) ){
          next_state_FSM2 = 26;
        }

        else if ( (BT == 4) ){
          next_state_FSM2 = 28;
        }
      } break;

      case 22: { //frente
        if ((BT == 11)  || stop_frente() ){
          next_state_FSM2 = 21;
        }
        else if ( (BT == 2) ){
          next_state_FSM2 = 23;
        }
        else if ( (BT == 4)){
          next_state_FSM2 = 29;
        }
        
      } break;

      case 23: { //frente-direita
        if ( (BT == 12)  ){
          next_state_FSM2 = 22;
        }
        else if ( (BT == 11)  ){
          next_state_FSM2 = 24;
        }  
        else if ( stop_frente()  ){
          next_state_FSM2 = 21;
        }  
        
      

      } break;

      case 24: { //direita forte
        if (  (BT == 12)   ){
          next_state_FSM2 = 21;
        }
        else if ( (BT == 1)  ){
          next_state_FSM2 = 23;
        } 
        else if ( (BT == 3)  ){
          next_state_FSM2 = 25;
        } 

      } break;

      case 25: { //trÃ¡s- esq
        if (  (BT == 12)  ){
          next_state_FSM2 = 26;
        }  
        else if ( (BT == 13)  ){
          next_state_FSM2 = 24;
        } 
    
      } break;

      case 26: { //trÃ¡s
        if (  (BT == 13)   ){
          next_state_FSM2 = 21;
        }
        else if ( (BT == 2)  ){
          next_state_FSM2 = 25;
        } 
        else if ( (BT == 4)  ){
          next_state_FSM2 = 27;
        }     

      } break;

      case 27: { //trÃ¡s- dir
        if (  (BT == 13)  ){
          next_state_FSM2 = 28;
        }  
        else if ( (BT == 14)  ){
          next_state_FSM2 = 26;
        }    

      } break;

      case 28: { //esquerda forte
        if (  (BT == 14)   ){
          next_state_FSM2 = 21;
        }
        else if ( (BT == 1)  ){
          next_state_FSM2 = 29;
        } 
        else if ( (BT == 3)  ){
          next_state_FSM2 = 27;
        }  
     
      } break;

      case 29: { //frente-esquerda
        if (  (BT == 11)  ){
          next_state_FSM2 = 28;
        }  
        else if ( (BT == 14)  ){
          next_state_FSM2 = 22;
        }  
        else if ( stop_frente()  ){
          next_state_FSM2 = 21;
        }  
       
      } break;

        default:
        next_state_FSM2 = 20;  // in case of error goto state_FSM1 10
      break;
    }
    if (next_state_FSM2 != state_FSM2) {   // If needed, change state_FSM0
      state_FSM2 = next_state_FSM2;
    }
    if(state_FSM2 != 20 && (state_FSM0 != 1 ) ){ //mudou o modo
      state_FSM2=20;
    }



  

  //ATIVAÇAO DE MOTORES
  frente        (  (state_FSM2==22)    );
  vira_dir      (  (state_FSM2==23)    );
  vira_forte_dir(  (state_FSM2==24)    );
  vira_esq_tras (  (state_FSM2==25)    );
  tras          (  (state_FSM2==26)    );
  vira_dir_tras (  (state_FSM2==27)    );
  vira_forte_esq(  (state_FSM2==28)    );
  vira_esq      (  (state_FSM2==29)    );
  para          (  (state_FSM2==21)  ||  (state_FSM0==0)  );

 
  /*
    if (Tshow == 0) {      // It is time to show things...
      
      cli();
      Tshow = 5;
      sei(); 

      printf(" \t\t\t\t\t\t 1-%u \t 2-%u \t 3-%u \t 4-%u \t 5-%u \t \n", sensor_IR_1, sensor_IR_2, sensor_IR_3, sensor_IR_4, sensor_IR_5 );
      printf(" \t\t\t\t\t\t 1-%u \t 2-%u \t 3-%u \t 4-%u \t 5-%u \t \n", max_esq(), esq(), meio(), dir(), max_dir() );
      
      printf(" \n\n\t STOP-%u \t \n", stop_frente() );
      

      printf(" \t\t esq-%u \t dir-%u  \n", vel_esq, vel_dir );
      printf(" \t\t pid-%u \t erro-%u  \n", PID, ERRO);

      printf("state_FSM0: %u \n", state_FSM0);
      printf("state_FSM1: %u \n", state_FSM1);
      printf("state_FSM2: %u \n\n", state_FSM2 );      

    }
  */
    
  }
}

