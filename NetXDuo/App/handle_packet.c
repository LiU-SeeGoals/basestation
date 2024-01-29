#include "handle_packet.h"
#include <nrf24l01.h>

size_t allocator_taken = 0;

static void* memAlloc(void* allocatorData, size_t size) {
	static char buffer[1024];
	if (allocator_taken + size > 1024) {
		printf("Out of memory\n");
		return NULL;
	}
	size_t padding = 0;
	if (allocator_taken % 4 != 0) {
		padding = 4 - allocator_taken % 4;
	}
	void* ptr = buffer + allocator_taken + padding;
	allocator_taken += size + padding;
	return ptr;
}

static void memFree(void* allocatorData, void* ptr) {
	(void) allocatorData;
	(void) ptr;
}

ProtobufCAllocator allocator = {.alloc = &memAlloc, .free = &memFree};

UINT parse_packet(NX_PACKET* packet, PACKET_TYPE packet_type) {
  UINT ret = NX_SUCCESS;

  switch(packet_type) {
    case SSL_WRAPPER:
      {
        int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;
        SSLWrapperPacket* ssl_packet = NULL;
        ssl_packet = ssl__wrapper_packet__unpack(&allocator, length, packet->nx_packet_prepend_ptr);
        if (ssl_packet == NULL) {
          ret = NX_INVALID_PACKET;
        } else {
          printf("[NX] received msg\r\n");
        }
        // Free the memory.
        allocator_taken = 0;
        // TODO: we need to extract the interesting data that is supposed
        // to be sent to each robot, then queue them up and send them
      }
      break;
    case ROBOT_COMMAND:
      {
        int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;
        if (length > 32) {
          ret = NX_INVALID_PACKET;
        } else {
          NRF_EnterMode(NRF_MODE_STANDBY1);

          if (NRF_OK != NRF_Transmit(packet->nx_packet_prepend_ptr, length)) {
        	  printf("Failed transmit\n");
          }
          NRF_EnterMode(NRF_MODE_RX);
          printf("[RF] tx len: %d\r\n", length);
        }
      }
      break;
  }

  if (ret != NX_SUCCESS) {
    printf("[NX] Failed to parse UDP packet\r\n");
  }

  return ret;
}
