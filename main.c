/*
 * File:   main.c
 * Author: Manuel Calzadillas Valles
 * Contact: menycalzadil@hotmail.com
 * Phone: +52 6142898877
 * 
 * Created on 26 de octubre de 2020, 20:31
 * 
 * PINOUT DESCRIPTION PIC16F1619 20 pin DIP
 * 1:  VDD          |-u-|   11: VSS  
 * 2:  RA5          |   |   12: RA0 ICSPDAT ESC1
 * 3:  RA4 ESC4     |   |   13: RA1 ICSPCLK ESC2
 * 4:  RA3 MCLR     |   |   14: RA2 ESC3
 * 5:  RC5 CH3 RC   |   |   15: RC0
 * 6:  RC4 CH2 RC   |   |   16: RC1
 * 7:  RC3 CH1 RC   |   |   17: RC2
 * 8:  RC6 CH4 RC   |   |   18: RB4 GYRO SDA
 * 9:  RC7          |   |   19: RB5 DRONE BATTERY LECTURE
 * 10: RB7          |___|   20: RB6 GYRO SCLK
 */

// Inclusión de librerías necesarias para el funcionamiento del programa
#include "config.h"
//Un sensor con acc gyro y magnetometro 16 bits -32000 hasta 32000
signed int accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz;

//Variables previas
struct previous p;
//Cuatro esc con capacidad hasta 2000 unsigned 16 bits = 65535
unsigned esc1, esc2, esc3, esc4, count, TMR2H, count, TMR0H, tmrLoop, ch1, ch2, \
        ch3, ch4, voltage, setpoint;
unsigned char start;

void main(void) {     
    // Inicialización de configuraciones iniciales del PIC
    pic_init();
    
    //Inicialización de variables    
    start = 0;   
    LATCbits.LATC7 = 0;     // Pin C7 se apaga, es el LED indicador
    ch3 = 1000;             // Se inicializa el channel 3 para evitar errores 
    
    gyro_config();          // Se enciende el giroscopio y se configura
        
    // ch1 = ch2 = ch4 = 1500; // Channels 1, 2 and 4 are initialized to 1500
    while(1){
         // Se reinicia el contador del ciclo
        reset_timer_loop();
        // Condiciones iniciales
        if(!start){
            //Inicialización de variables                           
            esc1 = esc2 = esc3 = esc4 = 1000;
            p.p1 = p.p2 = p.p3 = p.p4 = 0;    
            start = 1;
        }
        if(start == 1 && ch4 > 1900 && ch3 < 1050){
            start = 2;
        }
        // Condición que se debe cumplir para encender el dron
        if(start == 2 && ch4 < 1600 && ch3 < 1050){
            start = 3;
        }      
        // Combinación para apagar el dron, palanca izquierda hasta la izquierda
        // abajo y luego al centro
        if(start == 3 && ch4 < 1050 && ch3 < 1050){
            start = 0;
        }
                
        // Se toman datos del sensor
        read_sensor(); 
        // Se calcula el PID
        calculate_pid();
        // Se envía las correcciones del PID a los ESC's
        balance_drone();        
        battery_compensation();        
        esc1 = esc2 = esc3 = esc4 = ch3;
        tmrLoop = (TMR0H << 8) | TMR0;
        // Para evitar que los motores piten mientras están apagados
        if(start < 3){                        
            PORTA |= 0b00010111;    // Se activa el puerto A para evitar que piten los ESC
            while((((TMR0H << 8) | TMR0) - tmrLoop) < 1000 || TMR0 < 0xE8);           
            PORTA &= 0b11101000;    // Se desactiva el puerto A
        }
        // Se arrancan motores
        else{
            if(esc1 < 1200)esc1 = 1200;     // To prevent propeller from stopping
            if(esc2 < 1200)esc2 = 1200;     // if esc value is less than 1200
            if(esc3 < 1200)esc3 = 1200;     // adjust it's value to 1200
            if(esc4 < 1200)esc4 = 1200;     // 
            PORTA |= 0b00010111;            // PORTA is set high in RA0, RA1, RA2 and RA4
            while((PORTA & 0b00010111) > 0){
                if((((TMR0H << 8) | TMR0) - tmrLoop) > esc1)PORTA &= 0b11111110;
                if((((TMR0H << 8) | TMR0) - tmrLoop) > esc2)PORTA &= 0b11111101;
                if((((TMR0H << 8) | TMR0) - tmrLoop) > esc3)PORTA &= 0b11111011;
                if((((TMR0H << 8) | TMR0) - tmrLoop) > esc4)PORTA &= 0b11101111;
            }
        }       
        while(TMR0H < 0x4E || TMR0 < 20);
    }    
    SLEEP();
}
