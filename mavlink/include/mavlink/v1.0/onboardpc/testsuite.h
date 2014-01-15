/** @file
 *	@brief MAVLink comm protocol testsuite generated from onboardpc.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef ONBOARDPC_TESTSUITE_H
#define ONBOARDPC_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_onboardpc(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_onboardpc(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_danger(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_danger_t packet_in = {
		93372036854775807ULL,
	}73.0,
	}101.0,
	}18067,
	}187,
	}254,
	}65,
	};
	mavlink_danger_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.danger_dir = packet_in.danger_dir;
        	packet1.avoid_dir = packet_in.avoid_dir;
        	packet1.danger_dist = packet_in.danger_dist;
        	packet1.sensor_id = packet_in.sensor_id;
        	packet1.danger_level = packet_in.danger_level;
        	packet1.avoid_level = packet_in.avoid_level;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_danger_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_danger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_danger_pack(system_id, component_id, &msg , packet1.time_usec , packet1.sensor_id , packet1.danger_level , packet1.danger_dist , packet1.danger_dir , packet1.avoid_level , packet1.avoid_dir );
	mavlink_msg_danger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_danger_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.sensor_id , packet1.danger_level , packet1.danger_dist , packet1.danger_dir , packet1.avoid_level , packet1.avoid_dir );
	mavlink_msg_danger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_danger_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_danger_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.sensor_id , packet1.danger_level , packet1.danger_dist , packet1.danger_dir , packet1.avoid_level , packet1.avoid_dir );
	mavlink_msg_danger_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_onboardpc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_danger(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ONBOARDPC_TESTSUITE_H
