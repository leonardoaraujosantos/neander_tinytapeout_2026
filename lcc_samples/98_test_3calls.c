/* Test: 3 function calls returning fixed values */
int func1(int a, int b) { return 15; }
int func2(int a, int b) { return 4095; }
int func3(int a, int b) { return 4080; }

int main(void) {
    int a = 255;
    int b = 3855;
    int r1 = func1(a, b);
    int r2 = func2(a, b);
    int r3 = func3(a, b);
    return r1 + r2 + r3;  /* Should be 8190 */
}
