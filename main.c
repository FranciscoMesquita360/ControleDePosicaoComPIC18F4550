/* Titulo: PID digital 
 * Versão: 2.1
 * Autor: Francisco Ivan
 * Data: 20/05/2018
 * Compilador: XC8
 * IDE: MplabX 4.10
 * Dispositivo: PIC18f4550
 * ======================================= Descrição ========================================== *
 * Este codigo realiza o controle PID de maneira discreta. Através dos dados de entrada         *
 * referência e feedback, ambos medidos pelo conversor A/D em AN1 e AN0, encontra-se o erro,    *
 * que então e usado como parametro para calcular um valor de pwm necessário para corrigi-lo    *
 * isso através da função: PID_set_PF(). O sentido de giro do motor e controlado através das    *
 * saidas digitais RA2, RA3.                                                                    *
 * ============================================================================================ *
 * ------lEDs-----------|-------SAIDAS------|----ENTRADAS---|
 * RB0: en1             |RA2: en1           |AN0: fb        |
 * RB1: en2             |RA3: en2           |AN1: ref       |
 * RB2: enG             |RA4: enG           |RE0: b1        |
 * RB3: if(erro0>500)   |CCP1: pwm          |RE1: b2        |
 * RB4: interrupcao     |PORTD: LCD         |RE2: b3        |
 */
// PIC18F4550 Configuration Bit Settings
#include <xc.h>             // compilador
#include <pic18f4550.h>     // dispositivo
#include	<stdio.h>       // para utilizar a funcao sprintf
#include	<stdarg.h>      // para utilizar a funcao sprintf
#include "config.h"         // configuracoes do pic
#include "LCD.h"            // funcoes do LCD

//================== defines e mapeamento de hardware =============//
#define _XTAL_FREQ 20000000     // constante usada pelo delay, clock
#define en1 PORTAbits.RA2       // en1 da ponte H, pino 4 do pic
#define en2 PORTAbits.RA3       // en2 da ponte H, pino 5 do pic
#define enG PORTAbits.RA4       // eneble geral da ponte H, pino 6 do pic
#define b1 PORTEbits.RE0        // botao b1 da plca 
#define b2 PORTEbits.RE1        // botao b2 da plca
#define b3 PORTEbits.RE2        // botao b3 da plca

#define SHIFT 256               // deslocamento <<8 para uso de ponto fixo

//==================== variaveis globais =========================//
float kp; // coeficiente da acao proporcional
float ki; // coeficiente da acao integral
float kd; // coeficiente da acao derivativa
float T = 0.005; // periodo de amostragem
long int k1; // coeficiente auxiliar, para uso de ponto fixo
long int k2; // coeficiente auxiliar, para uso de ponto fixo
long int k3; // coeficiente auxiliar, para uso de ponto fixo
long int erro0 = 0; // erro da amostra atual e(n)
long int erro1 = 0; // erro da amostra anterior e(n-1)
long int out0 = 0; // saida da funcao do PID
long int integral = 0; // parcela integral
long int derivativo = 0; // derivativo
long int ref = 0; // guarda o valor do potenciomentro de referencia       
long int fb = 0; // guarda o valor do potenciomentro de feedback
long int pwm = 0; // guarda valor de dutycicle do pwm
unsigned long int tick_time = 0; // base de tempo para delay
unsigned long int delay100 = 0; // variavel para delay com tick_time
unsigned long int delay500 = 0; // variavel para delay com tick_time
unsigned long int delay501 = 0; // variavel para delay com tick_time
unsigned long int delay502 = 0; // variavel para delay com tick_time
unsigned long int delay1000 = 0; // variavel para delay com tick_time
int aux = 0; // vaeivael auxiliar para imprimir valores positivos de erro
char buf[16]; // bufer usado no LCD
char buf2[16]; // bufer usado no LCD
int enS = 0; // varivel usada como eneble de software
unsigned char Lmax = 147;
unsigned char Lmin = 89;
unsigned char BandaMorta = 10;
bit d = 0;
//========================= funcoes =============================//

void ADC_init() {
    ADCON1 = 0b00001101; //Somente AN0 e AN1 como entrada analogica, referencia e realimentacao
    ADCON2 = 0b10001110; //justificado a direita, 64 fosc , 2 Tda
}// fim ADC_init

unsigned ADC_read(unsigned char canal) {
    ADCON0 = canal;
    __delay_us(10); // tempo para carregar capacitor interno
    ADCON0bits.GODONE = 1; // inicia a conversao
    while (ADCON0bits.GODONE); // espera o fim da conversao AD
    {
    }
    return ((ADRESH << 8) + ADRESL); // retorna um valor convertido de 0 - 1023
}// fim ADC_read

unsigned long int PID_set_PF() {

    erro0 = ref - fb; // calcula erro atual
    erro1 = erro0; // atualiza valores

    // calculo do PID 
    integral = integral + (erro0 * T); // parte integral
    derivativo = ((erro0 - erro1) / T); // parte derivativo
    out0 = ((k1 * erro0)+ (k2 * integral) + (k3 * derivativo));
    out0 = out0 >> 8; // deslocamento devido ao uso de ponto fixo

    if (out0 < 0x00)// caso negativo 
    {
        out0 = out0 * (-1); // multiplica por -1
    }
    if (out0 > 1023) // caso estoure
    {
        out0 = 1023; // saturar saida
    }
    if (out0 == 0)
        out0 = 0;
    return out0;
}// fim PID_set_PF
#ORG 0x1000, 0x3000

void interrupt high_priority interrupcoes(void) {
    if (INTCONbits.TMR0IF == 1) { // interrupcao por estouro de tmer0?
        PORTBbits.RB4 = !PORTBbits.RB4; // saida para medir a interrupcao
        INTCONbits.TMR0IF = 0; // limpa a flag
        tick_time++; // incremeta a base de tempo de delay
        if (enS == 1) { // enable de software em 1?
            fb = ADC_read(0b00000001); // ler valor potenciomentro de referencia AN0 pino 2
            __delay_us(10); // tempo para descarregar capacitor interno
            ref = ADC_read(0b00000101); // ler valor potenciomentro de realimentacao AN1 pino 3

            if (ref > Lmax) { // normaliza referencia
                ref = Lmax;
            }
            if (ref < Lmin) { // normaliza referencia
                ref = Lmin;
            }
            pwm = PID_set_PF(); // calcula valor do PID
            // 5v  sentido direito
            // 0v  sentido esquerdo

            // verifica sentido necessario para o motor 
            if (ref > fb) { // direita
                en1 = 1; // pino 4
                en2 = 0; // pino 5
                //enG = 1; // pino 6
                PORTBbits.RB0 = ~en1;
                PORTBbits.RB1 = ~en2;
                //PORTBbits.RB2 = ~enG;
            } else { // esquerda
                en1 = 0; // pino 4
                en2 = 1; // pino 5
                //enG = 1; // pino 6
                PORTBbits.RB0 = ~en1;
                PORTBbits.RB1 = ~en2;
                //PORTBbits.RB2 = ~enG;
            }
            if (fb >= Lmax) { // teste de limite superior
                if (en1 == 1) { //direita
                    pwm = 0;
                }
            }

            if (fb <= Lmin) {// teste de limite inferior
                if (en1 == 0) { // esquerda
                    pwm = 0;
                }
            }

            if (erro0 < 0x00) { // tratamento de valor para ser sempre possitivo
                aux = (erro0 * (-1));
            } else {
                aux = erro0;
            }

            if ((aux <= BandaMorta)) { // testa se entrou na banda morta
                PORTBbits.RB6 = 1;
                if (d == 0) { // para a primeira passagem
                    d = 1;
                    delay500 = tick_time + 50;
                }
                if (tick_time > delay500) {// testa se passou 500ms na bamda morta
                    pwm = 0;
                    PORTBbits.RB6 = 0;
                    en1 = 0; // pino 4
                    en2 = 0; // pino 5
                    //enG = 0;
                    PORTBbits.RB0 = ~en1;
                    PORTBbits.RB1 = ~en2;
                    //PORTBbits.RB2 = ~enG;
                    // seta o PWM (0-1023) em CCPR1L:CCPxCON<5:4>
                    CCPR1L = pwm >> 2; // 8 bits mais significativos
                    CCP1CONbits.DC1B1 = pwm >> 1; // segundo bit menos significativo
                    CCP1CONbits.DC1B0 = pwm; // bit menos significativo
                }

            } else {
                //enG = 1;
                //PORTBbits.RB2 = ~enG;
                PORTBbits.RB6 = 0;
                d = 0;
            }

            if ((aux > 500)&&(tick_time > delay501)) { // verifica se o erro ultrapassou valores de segurança
                delay501 = tick_time + 200;
                pwm = 0;
                en1 = 0;
                en2 = 0;
                //  enG = 0;
                PORTBbits.RB0 = ~en1;
                PORTBbits.RB1 = ~en2;
            }

            // seta o PWM (0-1023) em CCPR1L:CCPxCON<5:4>
            CCPR1L = pwm >> 2; // 8 bits mais significativos
            CCP1CONbits.DC1B1 = pwm >> 1; // segundo bit menos significativo
            CCP1CONbits.DC1B0 = pwm; // bit menos significativo

            TMR0L = 22 + TMR0L; // interrupcao a cada 0.005 segundos
        } else { // se nao, enable de software nao habilitado
            en1 = 0;
            en2 = 0;
            //  enG = 0;
            PORTBbits.RB0 = ~en1;
            PORTBbits.RB1 = ~en2;
            //PORTBbits.RB2 = ~enG;

        }// fim if(enS)
    }// fim if(INTCONbits.TMR0IF)
}// fim interrupt high_priority

void setup() {
    CMCON = 0x07; // desabilita os comparadores internos
    // configura saidas e entradas
    TRISA = 0b00000011; // somente RA0,RA1 como entrada analogica, demais pinos saida
    TRISB = 0x00; // todo o PORTB como saida
    TRISC = 0x00; // todo o PORTC como saida, onde pino 17 RC2/CCP1 sera a saida PWM 
    TRISD = 0x00; // todo o PORTD como saida, onde se encotra conectado o LCD
    TRISE = 0x0F; // todo o PORTE como saida
    PORTB = 0xFF; // inicia o PORTB todo em alto
    ADC_init(); // inicia configuracoes do ADC
    Lcd_Init(); // inicia configurações do LCD
    ADCON0bits.ADON = 1; // liga o modulo adc  
    PORTDbits.RD2 = 0; // pino onde se encontra o RW do LCD, para escrita deve estar em nivel logico baixo
    // configuracao PWM
    T2CON = 0x07; // Liga timer2 e ativa prescaler 16
    PR2 = 0x7F; // Carrega PR2
    CCP1CON = 0x3C;
    CCPR1L = 0x00;
    CCP1CONbits.DC1B0 = 0;
    CCP1CONbits.DC1B1 = 0;
}// fim setup


//====================== funcao principal =======================//

void main() { // inicio main
    enG = 0; // desabilita enable geral do driver
    en1 = 0; // saida digital RA2 para ponte H,
    en2 = 0; // saida digital RA3 para ponte H
    setup(); // faz set up das configuracoes

    PORTDbits.RD2 = 0; // pino onde se encontra o RW do LCD, para escrita deve estar em nivel logico baixo

    // vermelho out B
    // 274
    // 130 
    // coeficientes do PID
    kp = 50;  // 50 limite mecanico 0,8
    ki = 0;
    kd = 0;
    // calculo dos coeficientes para ponto fixo
    k1 = (kp) * SHIFT;
    k2 = (ki) * SHIFT;
    k3 = (kd) * SHIFT;

    // configuracoes TIMER0
    T0CONbits.TMR0ON = 1; // liga timer0
    T0CONbits.T08BIT = 1; // selecioma modo 8 bits
    T0CONbits.T0CS = 0; // incrementa com clock interno, contador de tempo
    T0CONbits.T0SE = 0; // nao utilizado
    T0CONbits.PSA = 0; // prescaler habilitado
    T0CONbits.T0PS = 0b111; // prescaler de 256
    TMR0L = 22;
    // configuracoes de interrupcao
    INTCONbits.TMR0IE = 1; // habiliata interrupcao por estouro de timer0
    INTCON2bits.TMR0IP = 1; // interrupcao de timer0 de alta prioridade
    RCONbits.IPEN = 0; // desabilita prioridade em niveis de interrupcao
    INTCONbits.GIE_GIEH = 1; // habilita interrupcao global
    INTCONbits.PEIE_GIEL = 0; // desabilita todas as interrupcoes por perifericos
    PORTBbits.RB6 = 0;
    int estado = 0;
    while (1) { // inicio loop
        if (b1 == 0) { // seta valor de enable de software
            enS = 1;
        }
        if (b2 == 0) { // zera valor de enable de software
            enS = 0;
        }
        if (b3 == 0) {
            PORTBbits.RB7 = 1;
            delay502 = tick_time + 100;
        }
        if (tick_time > delay502) {
            PORTBbits.RB7 = 0;
        }
        if (tick_time > delay1000) {
             delay1000 = tick_time + 400;
            PORTBbits.RB5 = ~PORTBbits.RB5;
        }

        if (tick_time > delay100) { // atualiza display

            delay100 = tick_time + 100;
            Lcd_Clear();
            sprintf(buf, "r:%i", ref);
            Lcd_Set_Cursor(1, 1);
            Lcd_Write_String(buf);
            sprintf(buf, "p:%i", pwm);
            Lcd_Set_Cursor(1, 7);
            Lcd_Write_String(buf);
            sprintf(buf2, "f:%i", fb);
            Lcd_Set_Cursor(2, 1);
            Lcd_Write_String(buf2);
            sprintf(buf2, "e:%i", aux);
            Lcd_Set_Cursor(2, 7);
            Lcd_Write_String(buf2);
        }// fim dispaly
    }// fim loop
}// fim main