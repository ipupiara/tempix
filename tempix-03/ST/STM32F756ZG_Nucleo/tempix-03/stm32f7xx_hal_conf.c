
// under construction
// the idea is later to put all stm32Fxxx processor specific hw dependend code here 


typedef struct
 {
   __IO uint32_t ISR;   /*!< DMA interrupt status register */
   __IO uint32_t Reserved0;
   __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
 } DMA_Base_Registers_Copied;
 

