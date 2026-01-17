/*
 * 18_nested.c - Nested function calls test
 * Tests: Deeply nested function calls, expression evaluation
 * Expected: 120
 */

int double_val(int x) {
    return x + x;
}

int add_ten(int x) {
    return x + 10;
}

int main(void) {
    /* Nested: add_ten(double_val(double_val(5))) */
    /* double_val(5) = 10 */
    /* double_val(10) = 20 */
    /* add_ten(20) = 30 */
    int r1 = add_ten(double_val(double_val(5)));  /* 30 */

    /* More nesting: double_val(double_val(add_ten(5))) */
    /* add_ten(5) = 15 */
    /* double_val(15) = 30 */
    /* double_val(30) = 60 */
    int r2 = double_val(double_val(add_ten(5)));  /* 60 */

    /* Simple nested: add_ten(add_ten(add_ten(0))) = 30 */
    int r3 = add_ten(add_ten(add_ten(0)));  /* 30 */

    return r1 + r2 + r3;  /* 30 + 60 + 30 = 120 */
}
