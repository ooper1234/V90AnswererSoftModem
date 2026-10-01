/* Check the complete-frame send and forced partial-write recovery. */
#include <sys/socket.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
static unsigned char sent[1024];
static size_t sent_len, limit;
static int calls;
static ssize_t capture_send(int fd, const void *data, size_t len, int flags)
{
    (void) fd;
    (void) flags;
    if (limit && len > limit) len = limit;
    assert(sent_len + len <= sizeof(sent));
    memcpy(sent + sent_len, data, len);
    sent_len += len;
    calls++;
    return (ssize_t) len;
}
#define send capture_send
#include "../src/ast_socket/sm_ast_socket.c"
#undef send

int main(void)
{
    int pair[2];
    sm_ast_socket_t socket;
    unsigned char payload[320];
    size_t i;
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, pair) == 0);
    memset(&socket, 0, sizeof(socket));
    socket.fd = pair[0];
    for (i = 0; i < sizeof(payload); i++) payload[i] = (unsigned char) i;
    assert(sm_ast_write(&socket, SM_AS_KIND_AUDIO, payload, sizeof(payload)) == 0);
    assert(calls == 1 && sent_len == 323);
    assert(sent[0] == SM_AS_KIND_AUDIO && sent[1] == 1 && sent[2] == 64);
    assert(memcmp(sent + 3, payload, sizeof(payload)) == 0);
    sent_len = 0; calls = 0; limit = 17;
    assert(sm_ast_write(&socket, SM_AS_KIND_AUDIO, payload, sizeof(payload)) == 0);
    assert(calls > 1 && sent_len == 323);
    assert(memcmp(sent + 3, payload, sizeof(payload)) == 0);
    sent_len = 0; calls = 0; limit = 0;
    assert(sm_ast_write(&socket, 0, NULL, 0) == 0);
    assert(calls == 1 && sent_len == 3);
    assert(sm_ast_write(&socket, 0, payload, 65536) == -1);
    assert(calls == 1);
    close(pair[0]); close(pair[1]);
    puts("AudioSocket complete frames and partial-write recovery: PASS");
    return 0;
}
