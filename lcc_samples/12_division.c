/*
 * 12_division.c - Division and modulo test
 * Tests: Integer division, modulo operator
 * Expected: 23 (100/7=14, 100%7=2, 50/10=5, 17%5=2, then 14+2+5+2=23)
 */

int divide(int a, int b) {
    return a / b;
}

int modulo(int a, int b) {
    return a % b;
}

int main(void) {
    int d1 = divide(100, 7);   /* 14 */
    int m1 = modulo(100, 7);   /* 2 */
    int d2 = divide(50, 10);   /* 5 */
    int m2 = modulo(17, 5);    /* 2 */
    return d1 + m1 + d2 + m2;  /* 14 + 2 + 5 + 2 = 23 */
}
