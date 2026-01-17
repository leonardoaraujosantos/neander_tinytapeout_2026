/* Test: all 3 bitwise ops combined like 09_bitwise */
int and_op(int a, int b) { return a & b; }
int or_op(int a, int b) { return a | b; }
int xor_op(int a, int b) { return a ^ b; }

int main(void) {
    int a = 255;     /* 0x00FF */
    int b = 3855;    /* 0x0F0F */
    int r1 = and_op(a, b);  /* 15 */
    int r2 = or_op(a, b);   /* 4095 */
    int r3 = xor_op(a, b);  /* 4080 */
    return r1 + r2 + r3;    /* 8190 */
}
