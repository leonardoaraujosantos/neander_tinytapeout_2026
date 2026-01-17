/*
 * 19_bitwise2.c - Additional bitwise operations
 * Tests: NOT operator, combined bitwise operations
 * Expected: 170
 */

int main(void) {
    int x = 0x55;    /* 85: 01010101 */
    int y = 0xAA;    /* 170: 10101010 */

    /* x | y should give 0xFF = 255 */
    int r1 = x | y;  /* 255 */

    /* x & y should give 0 */
    int r2 = x & y;  /* 0 */

    /* x ^ 0xFF should flip bits: 0xAA = 170 */
    int r3 = x ^ 0xFF;  /* 170 */

    /* ~x & 0xFF should give 0xAA = 170 */
    int r4 = (~x) & 0xFF;  /* 170 */

    /* Return r3 which should be 170 */
    return r3;  /* 170 */
}
