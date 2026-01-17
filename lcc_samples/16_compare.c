/*
 * 16_compare.c - Comparison operators test
 * Tests: All comparison operators (<, >, <=, >=, ==, !=)
 * Expected: 6 (all comparisons should be true, returning 1 each)
 */

int test_lt(int a, int b) {
    if (a < b) return 1;
    return 0;
}

int test_gt(int a, int b) {
    if (a > b) return 1;
    return 0;
}

int test_le(int a, int b) {
    if (a <= b) return 1;
    return 0;
}

int test_ge(int a, int b) {
    if (a >= b) return 1;
    return 0;
}

int test_eq(int a, int b) {
    if (a == b) return 1;
    return 0;
}

int test_ne(int a, int b) {
    if (a != b) return 1;
    return 0;
}

int main(void) {
    int count = 0;
    count = count + test_lt(5, 10);   /* 5 < 10: true */
    count = count + test_gt(10, 5);   /* 10 > 5: true */
    count = count + test_le(5, 5);    /* 5 <= 5: true */
    count = count + test_ge(10, 10);  /* 10 >= 10: true */
    count = count + test_eq(42, 42);  /* 42 == 42: true */
    count = count + test_ne(1, 2);    /* 1 != 2: true */
    return count;  /* 6 */
}
