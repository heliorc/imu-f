#include "includes.h"
#include "crc.h"

void crc_config(void)
{
  /* DeInit CRC peripheral */
  CRC_DeInit();
  
  /* Init the INIT register */
  CRC_SetInitRegister(0xFFFFFFFFU);
  
  /* Select 8-bit polynomial size */
  CRC_PolynomialSizeSelect(CRC_PolSize_32);
  
  // Set the polynomial coefficients use the default for the f4 with hal
  CRC_SetPolynomial(0x04C11DB7);
}

inline void append_crc_to_data(uint32_t* data, uint32_t size)
{
    data[size] = get_crc(data, size);;
}

#pragma GCC push_options
#pragma GCC optimize ("O3")
inline uint32_t get_crc(uint32_t* data, uint32_t size)
{
    CRC_ResetDR(); //reset data register
    for(uint32_t x=0; x<size; x++ )
    {
        CRC_CalcCRC(data[x]);
    }
    return CRC_GetCRC();
}
#pragma GCC pop_options