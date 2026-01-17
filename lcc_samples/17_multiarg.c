/*
 * 17_multiarg.c - Multiple function arguments test
 * Tests: Functions with 3, 4, and 5 parameters
 * Expected: 150
 */

int add3(int a, int b, int c) {
    return a + b + c;
}

int add4(int a, int b, int c, int d) {
    return a + b + c + d;
}

int add5(int a, int b, int c, int d, int e) {
    return a + b + c + d + e;
}

int main(void) {
    int r1 = add3(10, 20, 30);           /* 60 */
    int r2 = add4(1, 2, 3, 4);           /* 10 */
    int r3 = add5(10, 20, 30, 10, 10);   /* 80 */
    return r1 + r2 + r3;                 /* 60 + 10 + 80 = 150 */
}
