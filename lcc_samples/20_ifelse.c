/*
 * 20_ifelse.c - If-else chains and complex conditions
 * Tests: Multiple if-else, else-if chains
 * Expected: 100
 */

int classify(int x) {
    if (x < 0) {
        return 1;
    } else if (x == 0) {
        return 2;
    } else if (x < 10) {
        return 3;
    } else if (x < 100) {
        return 4;
    } else {
        return 5;
    }
}

int max3(int a, int b, int c) {
    if (a >= b && a >= c) {
        return a;
    } else if (b >= a && b >= c) {
        return b;
    } else {
        return c;
    }
}

int main(void) {
    int c1 = classify(-5);   /* 1 */
    int c2 = classify(0);    /* 2 */
    int c3 = classify(5);    /* 3 */
    int c4 = classify(50);   /* 4 */
    int c5 = classify(200);  /* 5 */

    int sum = c1 + c2 + c3 + c4 + c5;  /* 1+2+3+4+5 = 15 */

    int m1 = max3(10, 20, 15);   /* 20 */
    int m2 = max3(30, 5, 25);    /* 30 */
    int m3 = max3(1, 2, 35);     /* 35 */

    return sum + m1 + m2 + m3;   /* 15 + 20 + 30 + 35 = 100 */
}
