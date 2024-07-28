// CubeV2 dynamic indication
// arduino leonardo 5V


#include <avr/io.h>
#include <avr/interrupt.h>

  #define NUM_ITEM 9
  #define NUM_LAYER 3

  #define BUTTON_PIN 2

  #define NUM_MODE 1

  #define FLAG_PORTB 0b01000000
  #define FLAG_PORTD 0b10000000
  #define FLAG_PORTF 0b00100000

  // pins layers
  const uint8_t gLayerPT[NUM_LAYER] = {0x7 | FLAG_PORTF, // A0>PF7
                                       0x6 | FLAG_PORTF, // A1>PF6 
                                       0x5 | FLAG_PORTF};// A2>PF5 
  // pins columns
  const uint8_t gItemPT[NUM_ITEM] = {0x0 | FLAG_PORTD, // 3>PD0
                                     0x4 | FLAG_PORTD, // 4>PD4
                                     0x3 | FLAG_PORTB, // 14>PB3
                                     0x7 | FLAG_PORTD, // 6>PD7
                                     0x1 | FLAG_PORTB, // 15>PB1
                                     0x4 | FLAG_PORTB, // 8>PB4
                                     0x5 | FLAG_PORTB, // 9>PB5
                                     0x6 | FLAG_PORTB, // 10>PB6
                                     0x2 | FLAG_PORTB};// 16>PB2

  // cube cleaning mask bits
  volatile uint8_t gMaskPortB;
  volatile uint8_t gMaskPortD;
  volatile uint8_t gMaskPortF;  

  // renderer output buffer
  volatile uint8_t gBuffPortB[NUM_LAYER] = {0};
  volatile uint8_t gBuffPortD[NUM_LAYER] = {0};

  // temporary buffer for drawing 
  bool gFrameBuff[NUM_LAYER][NUM_ITEM] = {{1,0,0,0,0,0,0,0,0},{0,0,0,0,1,0,0,0,0},{0,0,0,0,0,0,0,0,1}};

  // button 
  volatile uint8_t gCurrentMode = 0;
  volatile bool gChangeMode = false;
  

  bool cubeDelay(unsigned long ms){
  
    unsigned long start = millis();
  
    do {
  
      if (gChangeMode){
        gChangeMode = false;
        return true;
      }
      
    } while (millis() - start < ms);
  
    return false;
  }


  uint8_t compileFlags(uint8_t *pd, uint8_t count, uint8_t flag){

    uint8_t ret = 0;

    for (uint8_t item = 0; item < count; item++)
      if (pd[item] & flag)
          ret |= 1 << (pd[item] & ~flag);

    return ret;
  }


  // Display video buffer 
  void cubeShow(){
    
    for (uint8_t layer = 0; layer < NUM_LAYER; layer++){

      gBuffPortD[layer] = 0;
      gBuffPortB[layer] = 0;
    
      for (uint8_t item = 0; item < NUM_ITEM; item++){

        if (gFrameBuff[layer][item]){

          if (gItemPT[item] & FLAG_PORTB){
             gBuffPortB[layer] |= 1 << (gItemPT[item] & ~FLAG_PORTB);
          } else 
          if (gItemPT[item] & FLAG_PORTD){
            gBuffPortD[layer] |= 1 << (gItemPT[item] & ~FLAG_PORTD);
          }
          
        } // if (gFrameBuff[layer][item])
        
      } // for (uint8_t item = 0; item < NUM_ITEM; item++)
      
    }

    
  }



void buttonInterrupt() {

  if (gCurrentMode < NUM_MODE)
    gCurrentMode++; else
    gCurrentMode = 0;
    
  gChangeMode = true;

}


void setup() {
  
  gMaskPortB = compileFlags(gItemPT, NUM_ITEM, FLAG_PORTB);
  gMaskPortD = compileFlags(gItemPT, NUM_ITEM, FLAG_PORTD); 
  gMaskPortF = compileFlags(gLayerPT, NUM_LAYER, FLAG_PORTF); 

  // set pins output
  DDRB |= gMaskPortB; 
  DDRD |= gMaskPortD;
  DDRF |= gMaskPortF;

  // CubeOffAll
  PORTB = PINB & ~gMaskPortB;
  PORTD = PIND & ~gMaskPortD;  
  PORTF = PINF | gMaskPortF;

  cli();
  TCCR1A = 0;
  TCCR1B = 0; 

  OCR1A = 50; // reg cmp. 
  TCCR1B |= (1 << WGM12); // enable CTC mode (reset by coincidence ) 

  // set prescaler 1024
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);

  TIMSK1 |= (1 << OCIE1A);  // enable interrupt by overlap
  sei();


  randomSeed(analogRead(10));

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, FALLING);

    
  //Serial.begin(9600);

/*
    EICRA &= ~(1 << ISC10);
    EICRA |= (1 << ISC11);    // set INT1 to trigger on high to low
    EIMSK |= (1 << INT1);     // Turns on INT1
*/  
}


/*
ISR (INT1_vect)
{
}
*/

volatile uint8_t countLayer;

ISR(TIMER1_COMPA_vect)
{
//uint8_t oldSREG = SREG;
//cli();

    // clear   
    PORTF |= gMaskPortF;
    PORTB &= ~gMaskPortB; 
    PORTD &= ~gMaskPortD;
  
    if (countLayer >= NUM_LAYER)
      countLayer = 0;

    // turn on columns 
    PORTB |= gBuffPortB[countLayer];
    PORTD |= gBuffPortD[countLayer];

    // turn on layer
    PORTF &= ~(1 << (gLayerPT[countLayer] & ~FLAG_PORTF));


//SREG = oldSREG;

    countLayer++;
     
}



void clearFrameBuff(){
  for (uint8_t layer = 0; layer < NUM_LAYER; layer++)
    for (uint8_t item = 0; item < NUM_ITEM; item++)
      gFrameBuff[layer][item] = 0;
}



void mode_0(){

  cubeShow();
  cubeDelay(1000);
  
}





void mode_1(){

  uint8_t rp = 0;
  int8_t rm = 8;

  
  while(true){

    clearFrameBuff();

    if (rp >= NUM_ITEM)
      rp = 0; 

    if (rm < 0)
      rm = 8; 
    
    gFrameBuff[0][rp] = 1;
    gFrameBuff[1][4] = 1;
    gFrameBuff[2][rm] = 1;   
    
    rp++;
    rm--;
 
    cubeShow();
    if (cubeDelay(500)) return;    
  }


  for (uint8_t layer = 0; layer < NUM_LAYER; layer++){
    
    for (uint8_t item = 0; item < NUM_ITEM; item++)
      gFrameBuff[layer][item] = 1;
  }

    

}



void mode_2(){
}

void mode_3(){
}

void mode_4(){
}


void loop() {
 

  switch (gCurrentMode){
    
    case 0: mode_0();
                 break;
                 
    case 1: mode_1();
                 break;
                 
    case 2: mode_2();
                 break;
                 
    case 3: mode_3();
                 break;  
                               
    case 4: mode_4();
                 break;    
  }

  


}
