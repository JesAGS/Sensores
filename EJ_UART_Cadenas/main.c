/**************************
 * MSP432 UART - Eco 12MHz.
 * Descripción: Programa para comunicación por puerto serie RS232. Se trabaja con las siguientes configuraciones:
 * CPU -> 12MHz MCKL/DCO
 * Reloj UART -> SMCLK 12MHz
 * Baudios -> 9600
 * Bits -> 8
 * Bit stop -> 1
 * Bit paridad -> No
 ***************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/*Variables globales*/
bool imprimir = false;
uint8_t i = 0, j=0,k=0;
uint32_t status2;
//uint8_t k= 0xAA;
char msg[30];

/* Página que permite calcular los valores del bloque EUSCI
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_ConfigV1 uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        6,                                     // BRDIV = 78
        8,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
        EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
};

static uint8_t Dato[3]={0x48, 0x4F, 0x4C, 0x41};


int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();

    P1 -> DIR |= BIT0;
        P1 -> OUT &= ~BIT0;

    //Configuración registro captura comparación en modo salida PWM toggle
    TIMER_A0 -> CCTL [0] = TIMER_A_CCTLN_OUTMOD_7 + TIMER_A_CCTLN_CCIE;

    //Cargamos base de tiempo                   //ACLK        /SMCLK
        TIMER_A0 -> CCR[0] = 32768;    //7.3 10.02 Hz   16 KHz        22.72 Hz

        TIMER_A0 -> CTL = TIMER_A_CTL_SSEL__ACLK + TIMER_A_CTL_MC__CONTINUOUS + TIMER_A_CTL_CLR+TIMER_A_CTL_IE;

    /* P1.2 and P1.3 en modo UART  */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* Configuración UART con base a la estructura de arriba */
    UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Habilitamos UART */
    UART_enableModule(EUSCI_A0_BASE);

    /* Interrupciones */

    NVIC -> ISER [0] |= 1 << ((TA0_0_IRQn) & 31);
    __enable_irq();
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);
    //Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();

    while(1)
    {
        /* Modo bajo consumo */
        //PCM_gotoLPM0();
        if(imprimir==true){             //si se ha habilitado la variable imprimir
            for(k=0;k<=30;k++){         //se comienzan a transmitir los datos almacenados
                                        //en el vector msg hasta que se detecte un salto de linea
                P1 -> OUT ^= BIT0;                        //o hasta que se hayan transmitido los 31 caracteres que podria contener
                UART_transmitData(EUSCI_A0_BASE,msg[k]);
                if(msg[k]==0x2C){
                    k=30;               //se asigna el valor 30 a k para terminar el ciclo
                }
            }
            imprimir=false;             //se regresa la variable imprimir a su valor predeterminado
        }

    }
}

/* Servicio interrupción EUSCIA_0 */
void EUSCIA0_IRQHandler(void)
{

    //float voltaje = 0;


    /* Se envia dato recibido por Tx */

    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);


    if((status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG))
    {
        //sprintf(msg,"%.3f,",voltaje);

      //  TIMER_A0 -> CTL = TIMER_A_CTL_IE;
        i=UART_receiveData(EUSCI_A0_BASE);  //Se recibe el dato y se guarda en la en la posicion
        msg[j]=i;                           //correspondiente del vector msg
        j++;                                //se aumenta el iterador para guardar en otra posición
/*
        if(j>=30){                          //si el iterador llega a 30 se reinicia
            j=0;
            imprimir=true;
        }
/*
        if(i==0x2C){                        //si se recibe un salto de linea

            //se habilita la variable para saber que ya se va a imprimir
            j=0;                            //el vector y se reinicia el iterador
        }*/



    }

}

void TA0_0_IRQHandler (void){
 //    status2 = Timer_A_getEnabledInterruptStatus(TIMER_A_CCTLN_CCIFG);
    TIMER_A0 -> CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //Limpiamos la bandera
    TIMER_A0 -> CCR[0] += 32768;        //Sumamos el offset para CCR0
            //P1 -> OUT ^= BIT0;
          //  if(msg[30] != 0x00){
                j=0;
                       imprimir=true;//}

}
