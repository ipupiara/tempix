#ifndef dma_tools
#define dma_tools

#include <uosii-includes.h>

INT16U  feCounter;
INT16U  teCounter;
INT16U  dmeCounter;

void incDMAErrorCounter(DMA_HandleTypeDef *hdma);

void clearDmaInterruptFlags(DMA_HandleTypeDef *hdma);

// method copied from stm32f7xx_hal_dma.c
void DMA_SetTransferConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
INT8U  dmaIsr(DMA_HandleTypeDef *hdma);
#endif
