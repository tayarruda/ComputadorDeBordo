//#include <avr/io.h>?/
#include <avr/interrupt.h>

#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

volatile int cont0 = 0, cont2 = 0, cont3=0;
volatile bool verificar_velocidade = false;
volatile bool verificar_combustivel = false;
int estado = -1;
void lerSensorCombustivel();
void lerSensorIgnicao();
void USART_init(void);
void USART_putstring(char* StringPtr);
unsigned char USART_receive(void);
void USART_send(unsigned char data);
uint8_t ad0_ignicao;
uint16_t ad1_velocidade;
uint16_t ad2_combustivel;
volatile bool piscaDireita = false;
volatile bool piscaEsquerda = false;
bool ler_sensor_combustivel;
bool ler_sensor_ignicao;
float teste;
char data[50];

int main(){
  // DDR: 1-> saida e 0->entrada
  DDRD |= 0b11110000; // Setando os pinos de 4 a 7 como saídas
  DDRD &= 0b11110011; // Setando os pinos de PD3 E PD2 como entrada p interurpcao externa

  // PORT: 1 -> alto e 0->baixo
  // PORT: 1-> pull-up (botao) e 0->bi-states (fontes)

  PORTD |= 0b00001100; // Definindo pinos PD3 E PD2 como pull up para interrupcao externa

  //Configurando interrupcao externa
  EICRA |= 0b00001010; // o sinal logico baixo gera uma interrupcao

  // Habilitando interrupcao do pino 2 (INT0/PD2) e pino 3 (INT1/PD3)
  EIMSK |= 0b00000011;
  
  //Configurando interrupcao TIMER2 conta ate 100ms
  TCCR2A |= 0b00000000;
  TCCR2B |= 0b00000011;
  TIMSK2 |= 0b00000001;

  //Configuração do ADC
  ADMUX |= 0b01000000;  //AVCC como referência, ADLAR = 0, MUX = 0000 (ADC0 como entrada); // ADC0 -> sinal de ignicao
  
  ADCSRA |= 0b10000111;// Enable do adc, Escalonador de Clock em 128;

  USART_init(); // Inicializando Comunicação Serial
  while(1){
    ADMUX  &= 0b11110000;
    ADMUX |= 0b01000000; //muda o MUX para o ADC0
    ADCSRA |= 0b01000000; // habilita o inicio da conversao
    while (!(ADCSRA & 0b00010000)); // ADIF setado pra 1 quando o circuito termina a conversão
    ad0_ignicao = ADC; // armazena o valor convertido na variavel ad0_ignicao
    if((uint8_t)(255*0.6) <= ad0_ignicao){
    // ---------- Estado Ligar Motor ----------------  ESTADO 0
        
        if(estado != 0){
          TIMSK2 |= 0b00000001;
          //Serial.println("ligado");
          sei();
          cont3 = 0; // reseta TIMER0
          PORTD |= 0b11001100; // liga LED do pino de ignição e os farois
          estado = 0;
        }
        if(verificar_combustivel){
          lerSensorCombustivel();
          verificar_combustivel = false;
        }
        lerSensorIgnicao();

        if(verificar_velocidade) // verifica a velocidade a cada 100ms
        {
           ADMUX  &= 0b11110000;
           ADMUX |= 0b01000001; // alterando o mux para o ADC1 (sensor de velocidade)
           ADCSRA |= 0b01000000; // habilita o inicio da conversao
           while (!(ADCSRA & 0b00010000)); // ADIF setado pra 1 quando o circuito termina a conversão
           ad1_velocidade = ADC; // armazena o valor convertido na variavel ad1_velocidade
           sprintf(data, "VELOCIDADE: %d \n",(int)ad1_velocidade);
           USART_putstring(data);
           verificar_velocidade = false;
        }

        
       
     }else if((uint8_t)(255*0.5) > ad0_ignicao && ad0_ignicao >= (uint8_t)(255*0.15) ){
     // Serial.println("farois");
        // ---------- Estado Ligar Faróis ----------------  ESTADO 1

        if(estado != 1){
          //Serial.println("farois");
          sei();
          estado = 1;
          PORTD |= 0b01001100; // Ligar Farol
          PORTD &= 0b01001100; //Desligando Ignição
          TIMSK2 |= 0b00000001;
        }
        
       
        if(verificar_combustivel){
          lerSensorCombustivel();
          verificar_combustivel = false;
        }

     }else{
    //  Serial.println("desligado");

        // ---------- Estado Desligado ---------------- ESTADO -1

        
        if(estado != -1){
          //Serial.println("desligado");
          estado = -1;
          PORTD &= 0b00001111; // desliga os elementos
          TIMSK2 &= 0b11111110; // desabilitando interrupcao timer2
          cli(); // desabilita interrupcao global
          
          piscaDireita = false;
          piscaEsquerda = false;

          // zerando contadores
          // cont0= 0; 
          cont2= 0;
        }
     }
    } 
  }


ISR(TIMER2_OVF_vect){
   cont2++;
   cont0++;
  if(cont3 <= 2500){
   cont3++;  
  } 
  else{
   PORTD &= 0b01111111;
   cont3 = 4000;   
  }
  
  if(cont2 == 250) // contagem de 100ms
  {
    cont2=0;
    verificar_velocidade=true;
  }
  if(cont0 == 2500) // contagem de 100ms
  {
    cont0=0;
    verificar_combustivel=true;
  }
 
}

ISR(INT0_vect){//Liga o pisca direito
    piscaDireita = !piscaDireita;
    if(!piscaDireita){
      PORTD &= 0b11101111;
    }
    else{
      PORTD |= 0b00010000;
    }
    piscaEsquerda = false;
    PORTD &= 0b11011111; // desliga pisca esquerdo
}

ISR(INT1_vect){//Liga o pisca esquerdo
    piscaEsquerda = !piscaEsquerda;
    piscaDireita = false;
    PORTD &= 0b11101111; // desliga pisca direito
    if(!piscaEsquerda){
      PORTD &= 0b11011111;
    }
    else{
      PORTD |= 0b00100000;  
    }
}

  
void lerSensorIgnicao(){
    ADMUX  &= 0b11110000;
    ADMUX |= 0b00000000; //muda o MUX para o ADC0
    ADCSRA |= 0b01000000; // habilita o inicio da conversao
    while (!(ADCSRA & 0b00010000)); // ADIF setado pra 1 quando o circuito termina a conversão
    ad0_ignicao = ADC; // armazena o valor convertido na variavel ad0_ignicao
}


void lerSensorCombustivel(){
    ADMUX  &= 0b11110000;
    ADMUX |= 0b01000010; // alterando o mux para o ADC2 (sensor de combustivel)
    ADCSRA |= 0b01000000; // habilita o inicio da conversao
    while (!(ADCSRA & 0b00010000)); // ADIF setado pra 1 quando o circuito termina a conversão
    ad2_combustivel = ADC; // armazena o valor convertido na variavel ad2_combustivel
    sprintf(data, "COMBUSTIVEL: %d \n",(int)ad2_combustivel);
    USART_putstring(data);
}

void USART_init(void) {

  UBRR0H = (uint8_t)(BAUD_PRESCALLER >> 8);
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (3 << UCSZ00);
}

unsigned char USART_receive(void) {

  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;

}

void USART_send( unsigned char data) {

  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;

}

void USART_putstring(char* StringPtr) {

  while (*StringPtr != 0x00) {
    USART_send(*StringPtr);
    StringPtr++;
  }

}
