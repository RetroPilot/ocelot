uint8_t crc_checksum(uint8_t *dat, int len, const uint8_t poly) {
  uint8_t crc = 0xFF;
  int i, j;
  for (i = len - 1; i >= 0; i--) {
    crc ^= dat[i];
    for (j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((crc << 1) ^ poly);
      }
      else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void gen_crc_lookup_table(uint8_t poly, uint8_t crc_lut[]) {

  uint8_t j;
  uint16_t i;

   for (i = 0; i < 256; i++) {
    uint8_t crc;
    crc = i;
    for (j = 0; j < 8; j++) {
      if ((crc & 0x80) != 0)
        crc = (uint8_t)((crc << 1) ^ poly);
      else
        crc <<= 1;
    }
    crc_lut[i] = crc;
  }
}

uint8_t lut_checksum(uint8_t *d, int l, uint8_t *table) {
  uint8_t crc = 0xFF; // Standard init value for CRC8
  // CRC the payload, skipping over the first byte where the CRC lives.
  for (int i = 1; i < l; i++) {
    crc ^= d[i] & 0xFF;
    crc = table[crc] ^ crc<<8;
  }
  crc = crc ^ 0xFF; //final xor
  return crc;
}