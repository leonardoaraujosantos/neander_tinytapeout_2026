/* Test: XOR operation */
int xor_op(int a, int b) { return a ^ b; }

int main(void) {
    int a = 255;     /* 0x00FF */
    int b = 3855;    /* 0x0F0F */
    int r = xor_op(a, b);  /* 0x00FF ^ 0x0F0F = 0x0FF0 = 4080 */
    return r;
}
