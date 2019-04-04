/* AUTOGENERATED FILE. DO NOT EDIT. */
#ifndef _MOCK_NRF_802154_DEBUG_H
#define _MOCK_NRF_802154_DEBUG_H

#ifndef __STATIC_INLINE
#define __STATIC_INLINE
#else
#undef __STATIC_INLINE
#define __STATIC_INLINE
#endif
#define SUPPRESS_INLINE_IMPLEMENTATION

#include <nrf_802154_debug.h>
#undef SUPPRESS_INLINE_IMPLEMENTATION
#undef __STATIC_INLINE
#define __STATIC_INLINE __STATIC_INLINE1

void mock_nrf_802154_debug_Init(void);
void mock_nrf_802154_debug_Destroy(void);
void mock_nrf_802154_debug_Verify(void);




#define nrf_802154_debug_init_Ignore() nrf_802154_debug_init_CMockIgnore()
void nrf_802154_debug_init_CMockIgnore(void);
#define nrf_802154_debug_init_Expect() nrf_802154_debug_init_CMockExpect(__LINE__)
void nrf_802154_debug_init_CMockExpect(UNITY_LINE_TYPE cmock_line);
typedef void (* CMOCK_nrf_802154_debug_init_CALLBACK)(int cmock_num_calls);
void nrf_802154_debug_init_StubWithCallback(CMOCK_nrf_802154_debug_init_CALLBACK Callback);

#endif
