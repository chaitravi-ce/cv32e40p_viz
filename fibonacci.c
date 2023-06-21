
#include <stdio.h>
#include <stdlib.h>

static int fib(int i) {
    return (i>1) ? fib(i-1) + fib(i-2) : i;
}

int main(int argc, char *argv[]) {

    int i;
    int num = (argc >= 2) ? atoi((const char *)argv[1]) : 20;

    printf("starting fib(%d)...\n", num);

    for(i=0; i<num; i++) {
        printf("fib(%d) = %d\n", i, fib(i));
    }

    printf("finishing...\n");

    return 0;
}