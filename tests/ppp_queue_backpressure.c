/* Include the real pump so the regression exercises its read policy. */
#include "../src/call/sm_call.c"
#include <assert.h>
#include <sys/socket.h>
int main(void) {
    int fd[2];
    unsigned char bytes[32], held[32];
    sm_call_t *c = calloc(1, sizeof(*c));
    assert(c && socketpair(AF_UNIX, SOCK_STREAM, 0, fd) == 0);
    assert(fcntl(fd[0], F_SETFL, O_NONBLOCK) == 0);
    c->ppp_fd = fd[0]; c->phase = SM_CALL_HANDSHAKE;
    for (int i=0;i<32;i++) bytes[i]=(unsigned char)(i+1);
    assert(write(fd[1], bytes, sizeof(bytes)) == sizeof(bytes));
    /* A full transmit queue must not consume a single relay byte. */
    for (int i=0;i<SM_BITQ_SIZE;i++) assert(sm_bitq_push(&c->txbits, 1)==0);
    pump_ppp(c);
    assert(recv(fd[0], held, sizeof(held), MSG_PEEK)==sizeof(held));
    assert(memcmp(held, bytes, sizeof(bytes))==0);
    /* One complete byte of capacity permits exactly one byte to be read. */
    for (int i=0;i<10;i++) assert(sm_bitq_pop(&c->txbits)==1);
    pump_ppp(c);
    assert(sm_bitq_count(&c->txbits)==SM_BITQ_SIZE);
    assert(recv(fd[0], held, sizeof(held), MSG_PEEK)==31);
    assert(memcmp(held, bytes+1,31)==0);
    /* Recovery drains the remaining relay bytes without losing their tail. */
    sm_bitq_init(&c->txbits); pump_ppp(c);
    assert(sm_bitq_bytes(&c->txbits)==31);
    for(int i=1;i<32;i++) {
        assert(sm_bitq_pop(&c->txbits)==0);
        for(int bit=0;bit<8;bit++) assert(sm_bitq_pop(&c->txbits)==((bytes[i]>>bit)&1));
        assert(sm_bitq_pop(&c->txbits)==1);
    }
    close(fd[0]);close(fd[1]);free(c);
    puts("PASS: full/partial retrain queues preserve unread relay bytes and byte order");
    return 0;
}
