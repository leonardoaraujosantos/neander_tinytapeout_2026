/*
 * 15_array_idx.c - Array summation (constant indices)
 * Tests: Larger array with constant index access
 * NOTE: Variable index access not yet supported in LCC backend
 * Expected: 45 (0+1+2+3+4+5+6+7+8+9 = 45)
 */

int arr[10];

int main(void) {
    /* Initialize with constant indices */
    arr[0] = 0;
    arr[1] = 1;
    arr[2] = 2;
    arr[3] = 3;
    arr[4] = 4;
    arr[5] = 5;
    arr[6] = 6;
    arr[7] = 7;
    arr[8] = 8;
    arr[9] = 9;

    /* Sum with constant indices */
    return arr[0] + arr[1] + arr[2] + arr[3] + arr[4] +
           arr[5] + arr[6] + arr[7] + arr[8] + arr[9];  /* 45 */
}
