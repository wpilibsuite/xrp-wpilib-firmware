// Exercise the actual bundled BTstack channel allocator, not the radio mock.
#include "btstack_config.h"
#undef ENABLE_LOG_INFO
#undef ENABLE_LOG_DEBUG
#undef ENABLE_LOG_ERROR
#include "l2cap.c"

#include <assert.h>
#include <stdio.h>

static l2cap_channel_t channel_pool[MAX_NR_L2CAP_CHANNELS];
static bool channel_used[MAX_NR_L2CAP_CHANNELS];

l2cap_channel_t *btstack_memory_l2cap_channel_get(void) {
    for (unsigned i = 0; i < MAX_NR_L2CAP_CHANNELS; ++i) {
        if (!channel_used[i]) {
            channel_used[i] = true;
            memset(&channel_pool[i], 0, sizeof(channel_pool[i]));
            return &channel_pool[i];
        }
    }
    return NULL;
}

void btstack_memory_l2cap_channel_free(l2cap_channel_t *channel) {
    unsigned index = (unsigned)(channel - channel_pool);
    assert(index < MAX_NR_L2CAP_CHANNELS && channel_used[index]);
    channel_used[index] = false;
}

int btstack_run_loop_remove_timer(btstack_timer_source_t *timer) {
    (void)timer;
    return false;
}

static l2cap_channel_t *open_channel(bd_addr_type_t address_type) {
    bd_addr_t address = {0};
    l2cap_channel_type_t type = address_type == BD_ADDR_TYPE_ACL
        ? L2CAP_CHANNEL_TYPE_CLASSIC : L2CAP_CHANNEL_TYPE_CHANNEL_CBM;
    l2cap_channel_t *channel = l2cap_create_channel_entry(
        NULL, type, address, address_type, 0x81, 512, LEVEL_0);
    assert(channel != NULL);
    if (address_type != BD_ADDR_TYPE_ACL) {
        assert(channel->local_cid >= 0x40 && channel->local_cid <= 0x7f);
    }
    assert(l2cap_get_channel_for_local_cid(channel->local_cid) == NULL);
    btstack_linked_list_add_tail(&l2cap_channels, (btstack_linked_item_t *)channel);
    return channel;
}

static void close_channel(l2cap_channel_t *channel) {
    btstack_linked_list_remove(&l2cap_channels, (btstack_linked_item_t *)channel);
    l2cap_free_channel_entry(channel);
}

int main(void) {
    // Match l2cap_init()'s initial counter and cross the LE boundary repeatedly.
    l2cap_local_source_cid = 0x40;
    unsigned wraps = 0;
    for (unsigned i = 0; i < 1024; ++i) {
        l2cap_channel_t *channel = open_channel(
            i % 2 ? BD_ADDR_TYPE_LE_RANDOM : BD_ADDR_TYPE_LE_PUBLIC);
        wraps += channel->local_cid == 0x40;
        close_channel(channel);
    }
    assert(wraps == 16);

    // Active channels must not be reused, including on either side of a wrap.
    l2cap_local_source_cid = 0x7f;
    l2cap_channel_t *first = open_channel(BD_ADDR_TYPE_LE_PUBLIC);
    l2cap_channel_t *second = open_channel(BD_ADDR_TYPE_LE_RANDOM);
    assert(first->local_cid == 0x40 && second->local_cid == 0x41);
    l2cap_local_source_cid = 0x7e;
    l2cap_channel_t *last = open_channel(BD_ADDR_TYPE_LE_PUBLIC);
    assert(last->local_cid == 0x7f);
    l2cap_channel_t *next = open_channel(BD_ADDR_TYPE_LE_RANDOM);
    assert(next->local_cid == 0x42);
    close_channel(first);
    close_channel(second);
    close_channel(last);
    close_channel(next);

    // Classic still gets its full range; an LE allocation after it resets into
    // the LE range even when the shared counter is already above 0x7f.
    l2cap_local_source_cid = 0x7f;
    l2cap_channel_t *classic = open_channel(BD_ADDR_TYPE_ACL);
    assert(classic->local_cid == 0x80);
    l2cap_channel_t *le = open_channel(BD_ADDR_TYPE_LE_PUBLIC);
    assert(le->local_cid == 0x40);
    close_channel(classic);
    close_channel(le);
    l2cap_local_source_cid = 0xfffe;
    classic = open_channel(BD_ADDR_TYPE_ACL);
    assert(classic->local_cid == 0x40);
    close_channel(classic);
    puts("BTstack CID allocation regressions passed (1024 reconnects)");
    return 0;
}
