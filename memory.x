MEMORY
{
  /* minimum; flash it either 512K, 1M or 2M; p. 55 */
  FLASH : ORIGIN = 0x400000, LENGTH = 256K
  FLASH_UPDATE : ORIGIN = ORIGIN(FLASH)+LENGTH(FLASH), LENGTH = 256K
  RAM : ORIGIN = 0x20400000, LENGTH = 256K /* 256K or 384K; p. 54 */
}

SECTIONS {
  __flash_update_start = ORIGIN(FLASH_UPDATE);
  __flash_update_len = LENGTH(FLASH_UPDATE);
}