#include <dma-tools.h>

// structure copied from stm32f7xx_hal_dma.c
typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;


void clearDmaInterruptFlags(DMA_HandleTypeDef *hdma)
{
	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
	regs->IFCR = 0x3FU << ((DMA_HandleTypeDef *)hdma)->StreamIndex;

}

INT8U  dmaIsr(DMA_HandleTypeDef *hdma)
{
	INT8U res = 0x00;
	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
	res = (INT8U)((regs->ISR >> ((DMA_HandleTypeDef *)hdma)->StreamIndex) & 0x3FU);
	return res;
}

void incDMAErrorCounter(DMA_HandleTypeDef *hdma)
{
	if (__HAL_DMA_GET_FLAG(hdma,__HAL_DMA_GET_TE_FLAG_INDEX(hdma))) {
		++ teCounter;
	}
	if (__HAL_DMA_GET_FLAG(hdma,__HAL_DMA_GET_FE_FLAG_INDEX(hdma))) {
		++ feCounter;
	}
	if (__HAL_DMA_GET_FLAG(hdma,__HAL_DMA_GET_DME_FLAG_INDEX(hdma))) {
		++ dmeCounter;
	}
}



// structure copied from stm32f7xx_hal_dma.c
void DMA_SetTransferConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Clear DBM bit */
  hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

  /* Configure DMA Stream data length */
  hdma->Instance->NDTR = DataLength;

  /* Memory to Peripheral */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    hdma->Instance->PAR = DstAddress;

    /* Configure DMA Stream source address */
    hdma->Instance->M0AR = SrcAddress;
  }
  /* Peripheral to Memory */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PAR = SrcAddress;

    /* Configure DMA Stream destination address */
    hdma->Instance->M0AR = DstAddress;
  }
}
