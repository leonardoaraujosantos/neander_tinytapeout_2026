/*
 * 14_negative.c - Negative number operations
 * Tests: Negative constants, signed arithmetic, negation
 * Expected: 50
 */

int negate(int x) {
    return -x;
}

int abs_val(int x) {
    if (x < 0) {
        return -x;
    }
    return x;
}

int main(void) {
    int a = -30;
    int b = 20;
    int sum = a + b;           /* -10 */
    int neg = negate(sum);     /* 10 */
    int av = abs_val(a);       /* 30 */
    int diff = b - a;          /* 20 - (-30) = 50 */
    return diff;               /* 50 */
}
