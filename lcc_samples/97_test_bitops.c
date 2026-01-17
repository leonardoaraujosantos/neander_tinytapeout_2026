/* Test: bitwise operations with fixed values */
int and_op(int a, int b) { return a & b; }
int or_op(int a, int b) { return a | b; }

int main(void) {
    int a = 255;     /* 0x00FF */
    int b = 3855;    /* 0x0F0F */
    int r1 = and_op(a, b);  /* 0x00FF & 0x0F0F = 0x000F = 15 */
    int r2 = or_op(a, b);   /* 0x00FF | 0x0F0F = 0x0FFF = 4095 */
    return r1 + r2;         /* 15 + 4095 = 4110 */
}
