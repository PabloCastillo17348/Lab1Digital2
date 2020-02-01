/* 
 * File:   CARRERASCAS17348FILE.c
 * Author: Pablo Castillo
 * Carne 17348
 * _Jesús_
 * Created on 24 de enero de 2020, 11:38 AM
 */
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = OFF        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
//#include 'p16f887.h'
#define _XTAL_FREQ 4000000  //RELIJ INTERNO DEL PIC

void inicio(void); //IDENTIFICACION DE FUNCION INICIO PARA CONFIG. DE PUERTOS


void inicio(void){
//CONFIGURACION DEL PUERTO A
    TRISA = 0;     //TODO EL PUERTO COMO SALIDAS DIGITALES
    ANSEL = 0;
    PORTA = 0;

//CONFIGURACION DEL PUERTO B
    ANSELH = 0;                 // TODO EL PUERTO COMO SALIDAS DIGITALES EXCEPTO
    TRISB = 0b00000111;      //BITS 0,1,2 PARA PUSH
    PORTB = 0b00000111;
    WPUB = 0b00000111;
    OPTION_REG = 0b00000111;

//CONFIGURACION DEL PUERTO C
    TRISC = 0;      //TODO EL PUERTO COMO SALIDAS PARA EL DISPLAY
    PORTC = 0b11111111;

//CONFIGURACION DEL PUERTO D
    TRISD = 0;     //TODO EL PUERTO D COMO SALIDAS
    PORTD = 0;
}

unsigned char estado = 0;   //DECLARACION DE VARIABLES
unsigned char ganador = 0;
unsigned char debounce = 0;
unsigned char debounce2 = 0;

void main(void) {
    
    inicio(); //SE LLAMA A LA FUNCION INICIO DE LA CONFIG. DE PUERTOS
    // . C D E G F A B
    unsigned char display[10] = {0x88, 0xBE, 0xC4, 0x94, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x67}; //TABLA DE DISPLAY
    
    if (PORTBbits.RB0==1 && estado == 0){ //REVISA QUE EL BOTON SE HAYA PRESIONADO Y ESTAR EN EL ESTADO DE INICIO
        PORTBbits.RB5 = 1;
        PORTC = display[3]; //SEMAFORO
        __delay_ms (1000);  //TIEMPO DE UN SEGUNDO PARA MOSTRAR LUZ DE SEMAFORO Y CONTEO EN DISPLAY
        PORTBbits.RB5 = 0;  //ENCENDIDO Y APAGADO DE LED DE SEMAFORO
        PORTBbits.RB4 = 1;
        PORTC = display[2]; //MUESTRA EN DISPLAY
        __delay_ms (1000);
        PORTBbits.RB4 = 0;
        PORTBbits.RB3 = 1;
        PORTC = display[1];
        __delay_ms (1000);
        PORTBbits.RB3 = 0;
        PORTC = display[0];
        __delay_ms (1000);
        PORTC = 0;    
        estado = estado + 1; //CAMBIO DE ESTADO   
    }
    else {
    }
    if (estado == 1){ //REVISA QUE SE ENCUENTRE EN EL ESTADO PARA EMPEZAR A JUGAR
        PORTC = display[0]; //SIEMPRE MUESTRA 0 EN EL DISPLAY
        while (ganador == 0){ //NO HAY UN GANADOR AUN
            //JUGADOR 1
            if (PORTBbits.RB2 == 1){
                //ANTIRREBOTE
                debounce = 1;
                __delay_ms (100);
            }
            if (debounce == 1 && PORTBbits.RB2 == 0){
                debounce = 0;
                if (PORTA == 0){    //REVISA QUE EL PUERTO ESTE APAGADO
                    PORTA = 1;      //DA EL VALOR DE 1 AL PUERTO
                }
                else{
                    PORTA = PORTA *2;   //MULTIPLICA POR 2 EL PUERTO PARA QUE SE ENCIENDA BIT POR BIT
                }
                if (PORTA == 0b10000000){ //COMPARACION PARA SABER SI YA GANO
                    estado = estado + 1;
                    ganador = 1;
                }
            }
            
            //ALGORITMO IDENTICO PARA EL SEGUNDO JUGADOR    
            //JUGADOR 2
            if (PORTBbits.RB1 == 1){
                //ANTIRREBOTE
                debounce2 = 1;
                __delay_ms (100);
            }
            if (debounce2 == 1 && PORTBbits.RB1 == 0){
                debounce2 = 0;
                if (PORTD == 0){
                    PORTD = 1;
                }
                else{
                    PORTD = PORTD *2;   
                }
                if (PORTD == 0b10000000){
                    estado = estado + 1;
                    ganador = 2;
                }
            }
        }
    }
    
    if (estado == 2){ //REVISA VARIABLE DE ESTADO
        if (ganador == 1){      //REVISA VARIABLE DE GANADOR
            __delay_ms(200);    //RETRASO MIENTRAS REVISA
            PORTBbits.RB7 = 1;  //ENCIENDE LA LED DEL JUGADOR 1
            __delay_ms(400);    //TIEMPO EN QUE PERMANECE ENCENDIDO EL LED
            PORTC = display[1]; //DESPLIEGA EL NUMERO DEL JUGADOR 1
            __delay_ms(400);    //TIEMPO EN QUE PERMANECE ENCENDIDO EL DISPLAY
            ganador = 0;        //CLEAR VARIABLES DE ESTADO Y GANADOR
            estado = 0;
        }
        //LO MISMO PARA JUGADOR 2
        if (ganador == 2){
            __delay_ms(200);
           PORTBbits.RB6 = 1;
            __delay_ms(400);
           PORTC = display[2];
           __delay_ms(400);
           ganador = 0;
           estado = 0;
        }
    }
    
    return;
}