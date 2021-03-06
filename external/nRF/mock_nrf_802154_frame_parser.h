/* AUTOGENERATED FILE. DO NOT EDIT. */
#ifndef _MOCK_NRF_802154_FRAME_PARSER_H
#define _MOCK_NRF_802154_FRAME_PARSER_H

#ifndef __STATIC_INLINE
#define __STATIC_INLINE
#else
#undef __STATIC_INLINE
#define __STATIC_INLINE
#endif
#define SUPPRESS_INLINE_IMPLEMENTATION

#include <nrf_802154_frame_parser.h>
#undef SUPPRESS_INLINE_IMPLEMENTATION
#undef __STATIC_INLINE
#define __STATIC_INLINE __STATIC_INLINE1

void mock_nrf_802154_frame_parser_Init(void);
void mock_nrf_802154_frame_parser_Destroy(void);
void mock_nrf_802154_frame_parser_Verify(void);




#define nrf_802154_frame_parser_dst_addr_is_extended_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_dst_addr_is_extended_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_dst_addr_is_extended_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, bool cmock_to_return);
#define nrf_802154_frame_parser_dst_addr_is_extended_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_dst_addr_is_extended_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_dst_addr_is_extended_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, bool cmock_to_return);
typedef bool (* CMOCK_nrf_802154_frame_parser_dst_addr_is_extended_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_dst_addr_is_extended_StubWithCallback(CMOCK_nrf_802154_frame_parser_dst_addr_is_extended_CALLBACK Callback);
#define nrf_802154_frame_parser_dst_addr_is_extended_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_dst_addr_is_extended_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_dst_addr_is_extended_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, bool cmock_to_return);
#define nrf_802154_frame_parser_dst_addr_is_extended_IgnoreArg_p_frame() nrf_802154_frame_parser_dst_addr_is_extended_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_dst_addr_is_extended_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_dst_addr_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_dst_addr_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_dst_addr_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_dst_addr_get_ExpectAndReturn(p_frame, p_dst_addr_extended, cmock_retval) nrf_802154_frame_parser_dst_addr_get_CMockExpectAndReturn(__LINE__, p_frame, p_dst_addr_extended, cmock_retval)
void nrf_802154_frame_parser_dst_addr_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, bool* p_dst_addr_extended, uint8_t* cmock_to_return);
typedef uint8_t* (* CMOCK_nrf_802154_frame_parser_dst_addr_get_CALLBACK)(const uint8_t* p_frame, bool* p_dst_addr_extended, int cmock_num_calls);
void nrf_802154_frame_parser_dst_addr_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_dst_addr_get_CALLBACK Callback);
#define nrf_802154_frame_parser_dst_addr_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, p_dst_addr_extended, p_dst_addr_extended_Depth, cmock_retval) nrf_802154_frame_parser_dst_addr_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, p_dst_addr_extended, p_dst_addr_extended_Depth, cmock_retval)
void nrf_802154_frame_parser_dst_addr_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, bool* p_dst_addr_extended, int p_dst_addr_extended_Depth, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_dst_addr_get_ReturnThruPtr_p_dst_addr_extended(p_dst_addr_extended) nrf_802154_frame_parser_dst_addr_get_CMockReturnMemThruPtr_p_dst_addr_extended(__LINE__, p_dst_addr_extended, sizeof(*p_dst_addr_extended))
#define nrf_802154_frame_parser_dst_addr_get_ReturnArrayThruPtr_p_dst_addr_extended(p_dst_addr_extended, cmock_len) nrf_802154_frame_parser_dst_addr_get_CMockReturnMemThruPtr_p_dst_addr_extended(__LINE__, p_dst_addr_extended, cmock_len * sizeof(*p_dst_addr_extended))
#define nrf_802154_frame_parser_dst_addr_get_ReturnMemThruPtr_p_dst_addr_extended(p_dst_addr_extended, cmock_size) nrf_802154_frame_parser_dst_addr_get_CMockReturnMemThruPtr_p_dst_addr_extended(__LINE__, p_dst_addr_extended, cmock_size)
void nrf_802154_frame_parser_dst_addr_get_CMockReturnMemThruPtr_p_dst_addr_extended(UNITY_LINE_TYPE cmock_line, bool* p_dst_addr_extended, int cmock_size);
#define nrf_802154_frame_parser_dst_addr_get_IgnoreArg_p_frame() nrf_802154_frame_parser_dst_addr_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_dst_addr_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_dst_addr_get_IgnoreArg_p_dst_addr_extended() nrf_802154_frame_parser_dst_addr_get_CMockIgnoreArg_p_dst_addr_extended(__LINE__)
void nrf_802154_frame_parser_dst_addr_get_CMockIgnoreArg_p_dst_addr_extended(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_dst_addr_offset_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_dst_addr_offset_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_dst_addr_offset_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_dst_addr_offset_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_dst_addr_offset_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_dst_addr_offset_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t cmock_to_return);
typedef uint8_t (* CMOCK_nrf_802154_frame_parser_dst_addr_offset_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_dst_addr_offset_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_dst_addr_offset_get_CALLBACK Callback);
#define nrf_802154_frame_parser_dst_addr_offset_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_dst_addr_offset_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_dst_addr_offset_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_dst_addr_offset_get_IgnoreArg_p_frame() nrf_802154_frame_parser_dst_addr_offset_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_dst_addr_offset_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_dst_panid_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_dst_panid_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_dst_panid_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_dst_panid_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_dst_panid_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_dst_panid_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t* cmock_to_return);
typedef uint8_t* (* CMOCK_nrf_802154_frame_parser_dst_panid_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_dst_panid_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_dst_panid_get_CALLBACK Callback);
#define nrf_802154_frame_parser_dst_panid_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_dst_panid_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_dst_panid_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_dst_panid_get_IgnoreArg_p_frame() nrf_802154_frame_parser_dst_panid_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_dst_panid_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_dst_panid_offset_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_dst_panid_offset_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_dst_panid_offset_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_dst_panid_offset_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_dst_panid_offset_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_dst_panid_offset_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t cmock_to_return);
typedef uint8_t (* CMOCK_nrf_802154_frame_parser_dst_panid_offset_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_dst_panid_offset_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_dst_panid_offset_get_CALLBACK Callback);
#define nrf_802154_frame_parser_dst_panid_offset_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_dst_panid_offset_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_dst_panid_offset_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_dst_panid_offset_get_IgnoreArg_p_frame() nrf_802154_frame_parser_dst_panid_offset_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_dst_panid_offset_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_dst_addr_end_offset_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_dst_addr_end_offset_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_dst_addr_end_offset_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_dst_addr_end_offset_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_dst_addr_end_offset_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_dst_addr_end_offset_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t cmock_to_return);
typedef uint8_t (* CMOCK_nrf_802154_frame_parser_dst_addr_end_offset_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_dst_addr_end_offset_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_dst_addr_end_offset_get_CALLBACK Callback);
#define nrf_802154_frame_parser_dst_addr_end_offset_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_dst_addr_end_offset_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_dst_addr_end_offset_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_dst_addr_end_offset_get_IgnoreArg_p_frame() nrf_802154_frame_parser_dst_addr_end_offset_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_dst_addr_end_offset_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_src_addr_is_extended_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_src_addr_is_extended_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_src_addr_is_extended_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, bool cmock_to_return);
#define nrf_802154_frame_parser_src_addr_is_extended_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_src_addr_is_extended_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_src_addr_is_extended_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, bool cmock_to_return);
typedef bool (* CMOCK_nrf_802154_frame_parser_src_addr_is_extended_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_src_addr_is_extended_StubWithCallback(CMOCK_nrf_802154_frame_parser_src_addr_is_extended_CALLBACK Callback);
#define nrf_802154_frame_parser_src_addr_is_extended_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_src_addr_is_extended_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_src_addr_is_extended_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, bool cmock_to_return);
#define nrf_802154_frame_parser_src_addr_is_extended_IgnoreArg_p_frame() nrf_802154_frame_parser_src_addr_is_extended_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_src_addr_is_extended_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_src_addr_is_short_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_src_addr_is_short_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_src_addr_is_short_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, bool cmock_to_return);
#define nrf_802154_frame_parser_src_addr_is_short_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_src_addr_is_short_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_src_addr_is_short_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, bool cmock_to_return);
typedef bool (* CMOCK_nrf_802154_frame_parser_src_addr_is_short_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_src_addr_is_short_StubWithCallback(CMOCK_nrf_802154_frame_parser_src_addr_is_short_CALLBACK Callback);
#define nrf_802154_frame_parser_src_addr_is_short_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_src_addr_is_short_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_src_addr_is_short_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, bool cmock_to_return);
#define nrf_802154_frame_parser_src_addr_is_short_IgnoreArg_p_frame() nrf_802154_frame_parser_src_addr_is_short_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_src_addr_is_short_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_src_addr_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_src_addr_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_src_addr_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_src_addr_get_ExpectAndReturn(p_frame, p_src_addr_extended, cmock_retval) nrf_802154_frame_parser_src_addr_get_CMockExpectAndReturn(__LINE__, p_frame, p_src_addr_extended, cmock_retval)
void nrf_802154_frame_parser_src_addr_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, bool* p_src_addr_extended, uint8_t* cmock_to_return);
typedef uint8_t* (* CMOCK_nrf_802154_frame_parser_src_addr_get_CALLBACK)(const uint8_t* p_frame, bool* p_src_addr_extended, int cmock_num_calls);
void nrf_802154_frame_parser_src_addr_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_src_addr_get_CALLBACK Callback);
#define nrf_802154_frame_parser_src_addr_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, p_src_addr_extended, p_src_addr_extended_Depth, cmock_retval) nrf_802154_frame_parser_src_addr_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, p_src_addr_extended, p_src_addr_extended_Depth, cmock_retval)
void nrf_802154_frame_parser_src_addr_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, bool* p_src_addr_extended, int p_src_addr_extended_Depth, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_src_addr_get_ReturnThruPtr_p_src_addr_extended(p_src_addr_extended) nrf_802154_frame_parser_src_addr_get_CMockReturnMemThruPtr_p_src_addr_extended(__LINE__, p_src_addr_extended, sizeof(*p_src_addr_extended))
#define nrf_802154_frame_parser_src_addr_get_ReturnArrayThruPtr_p_src_addr_extended(p_src_addr_extended, cmock_len) nrf_802154_frame_parser_src_addr_get_CMockReturnMemThruPtr_p_src_addr_extended(__LINE__, p_src_addr_extended, cmock_len * sizeof(*p_src_addr_extended))
#define nrf_802154_frame_parser_src_addr_get_ReturnMemThruPtr_p_src_addr_extended(p_src_addr_extended, cmock_size) nrf_802154_frame_parser_src_addr_get_CMockReturnMemThruPtr_p_src_addr_extended(__LINE__, p_src_addr_extended, cmock_size)
void nrf_802154_frame_parser_src_addr_get_CMockReturnMemThruPtr_p_src_addr_extended(UNITY_LINE_TYPE cmock_line, bool* p_src_addr_extended, int cmock_size);
#define nrf_802154_frame_parser_src_addr_get_IgnoreArg_p_frame() nrf_802154_frame_parser_src_addr_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_src_addr_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_src_addr_get_IgnoreArg_p_src_addr_extended() nrf_802154_frame_parser_src_addr_get_CMockIgnoreArg_p_src_addr_extended(__LINE__)
void nrf_802154_frame_parser_src_addr_get_CMockIgnoreArg_p_src_addr_extended(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_src_addr_offset_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_src_addr_offset_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_src_addr_offset_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_src_addr_offset_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_src_addr_offset_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_src_addr_offset_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t cmock_to_return);
typedef uint8_t (* CMOCK_nrf_802154_frame_parser_src_addr_offset_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_src_addr_offset_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_src_addr_offset_get_CALLBACK Callback);
#define nrf_802154_frame_parser_src_addr_offset_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_src_addr_offset_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_src_addr_offset_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_src_addr_offset_get_IgnoreArg_p_frame() nrf_802154_frame_parser_src_addr_offset_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_src_addr_offset_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_src_panid_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_src_panid_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_src_panid_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_src_panid_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_src_panid_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_src_panid_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t* cmock_to_return);
typedef uint8_t* (* CMOCK_nrf_802154_frame_parser_src_panid_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_src_panid_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_src_panid_get_CALLBACK Callback);
#define nrf_802154_frame_parser_src_panid_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_src_panid_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_src_panid_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_src_panid_get_IgnoreArg_p_frame() nrf_802154_frame_parser_src_panid_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_src_panid_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_src_panid_offset_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_src_panid_offset_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_src_panid_offset_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_src_panid_offset_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_src_panid_offset_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_src_panid_offset_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t cmock_to_return);
typedef uint8_t (* CMOCK_nrf_802154_frame_parser_src_panid_offset_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_src_panid_offset_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_src_panid_offset_get_CALLBACK Callback);
#define nrf_802154_frame_parser_src_panid_offset_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_src_panid_offset_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_src_panid_offset_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_src_panid_offset_get_IgnoreArg_p_frame() nrf_802154_frame_parser_src_panid_offset_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_src_panid_offset_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_mhr_parse_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_mhr_parse_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_mhr_parse_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, bool cmock_to_return);
#define nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(p_frame, p_fields, cmock_retval) nrf_802154_frame_parser_mhr_parse_CMockExpectAndReturn(__LINE__, p_frame, p_fields, cmock_retval)
void nrf_802154_frame_parser_mhr_parse_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, nrf_802154_frame_parser_mhr_data_t* p_fields, bool cmock_to_return);
typedef bool (* CMOCK_nrf_802154_frame_parser_mhr_parse_CALLBACK)(const uint8_t* p_frame, nrf_802154_frame_parser_mhr_data_t* p_fields, int cmock_num_calls);
void nrf_802154_frame_parser_mhr_parse_StubWithCallback(CMOCK_nrf_802154_frame_parser_mhr_parse_CALLBACK Callback);
#define nrf_802154_frame_parser_mhr_parse_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, p_fields, p_fields_Depth, cmock_retval) nrf_802154_frame_parser_mhr_parse_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, p_fields, p_fields_Depth, cmock_retval)
void nrf_802154_frame_parser_mhr_parse_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, nrf_802154_frame_parser_mhr_data_t* p_fields, int p_fields_Depth, bool cmock_to_return);
#define nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(p_fields) nrf_802154_frame_parser_mhr_parse_CMockReturnMemThruPtr_p_fields(__LINE__, p_fields, sizeof(*p_fields))
#define nrf_802154_frame_parser_mhr_parse_ReturnArrayThruPtr_p_fields(p_fields, cmock_len) nrf_802154_frame_parser_mhr_parse_CMockReturnMemThruPtr_p_fields(__LINE__, p_fields, cmock_len * sizeof(*p_fields))
#define nrf_802154_frame_parser_mhr_parse_ReturnMemThruPtr_p_fields(p_fields, cmock_size) nrf_802154_frame_parser_mhr_parse_CMockReturnMemThruPtr_p_fields(__LINE__, p_fields, cmock_size)
void nrf_802154_frame_parser_mhr_parse_CMockReturnMemThruPtr_p_fields(UNITY_LINE_TYPE cmock_line, nrf_802154_frame_parser_mhr_data_t* p_fields, int cmock_size);
#define nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_frame() nrf_802154_frame_parser_mhr_parse_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_mhr_parse_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields() nrf_802154_frame_parser_mhr_parse_CMockIgnoreArg_p_fields(__LINE__)
void nrf_802154_frame_parser_mhr_parse_CMockIgnoreArg_p_fields(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_sec_ctrl_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_sec_ctrl_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_sec_ctrl_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_sec_ctrl_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_sec_ctrl_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_sec_ctrl_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t* cmock_to_return);
typedef uint8_t* (* CMOCK_nrf_802154_frame_parser_sec_ctrl_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_sec_ctrl_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_sec_ctrl_get_CALLBACK Callback);
#define nrf_802154_frame_parser_sec_ctrl_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_sec_ctrl_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_sec_ctrl_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_sec_ctrl_get_IgnoreArg_p_frame() nrf_802154_frame_parser_sec_ctrl_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_sec_ctrl_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_addressing_end_offset_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_addressing_end_offset_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_addressing_end_offset_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_addressing_end_offset_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_addressing_end_offset_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_addressing_end_offset_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t cmock_to_return);
typedef uint8_t (* CMOCK_nrf_802154_frame_parser_addressing_end_offset_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_addressing_end_offset_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_addressing_end_offset_get_CALLBACK Callback);
#define nrf_802154_frame_parser_addressing_end_offset_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_addressing_end_offset_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_addressing_end_offset_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_addressing_end_offset_get_IgnoreArg_p_frame() nrf_802154_frame_parser_addressing_end_offset_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_addressing_end_offset_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_sec_ctrl_offset_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_sec_ctrl_offset_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_sec_ctrl_offset_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_sec_ctrl_offset_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_sec_ctrl_offset_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_sec_ctrl_offset_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t cmock_to_return);
typedef uint8_t (* CMOCK_nrf_802154_frame_parser_sec_ctrl_offset_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_sec_ctrl_offset_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_sec_ctrl_offset_get_CALLBACK Callback);
#define nrf_802154_frame_parser_sec_ctrl_offset_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_sec_ctrl_offset_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_sec_ctrl_offset_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_sec_ctrl_offset_get_IgnoreArg_p_frame() nrf_802154_frame_parser_sec_ctrl_offset_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_sec_ctrl_offset_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_key_id_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_key_id_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_key_id_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_key_id_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_key_id_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_key_id_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t* cmock_to_return);
typedef uint8_t* (* CMOCK_nrf_802154_frame_parser_key_id_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_key_id_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_key_id_get_CALLBACK Callback);
#define nrf_802154_frame_parser_key_id_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_key_id_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_key_id_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_key_id_get_IgnoreArg_p_frame() nrf_802154_frame_parser_key_id_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_key_id_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_key_id_offset_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_key_id_offset_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_key_id_offset_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_key_id_offset_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_key_id_offset_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_key_id_offset_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t cmock_to_return);
typedef uint8_t (* CMOCK_nrf_802154_frame_parser_key_id_offset_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_key_id_offset_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_key_id_offset_get_CALLBACK Callback);
#define nrf_802154_frame_parser_key_id_offset_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_key_id_offset_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_key_id_offset_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_key_id_offset_get_IgnoreArg_p_frame() nrf_802154_frame_parser_key_id_offset_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_key_id_offset_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_dsn_suppress_bit_is_set_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_dsn_suppress_bit_is_set_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_dsn_suppress_bit_is_set_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, bool cmock_to_return);
#define nrf_802154_frame_parser_dsn_suppress_bit_is_set_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_dsn_suppress_bit_is_set_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_dsn_suppress_bit_is_set_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, bool cmock_to_return);
typedef bool (* CMOCK_nrf_802154_frame_parser_dsn_suppress_bit_is_set_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_dsn_suppress_bit_is_set_StubWithCallback(CMOCK_nrf_802154_frame_parser_dsn_suppress_bit_is_set_CALLBACK Callback);
#define nrf_802154_frame_parser_dsn_suppress_bit_is_set_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_dsn_suppress_bit_is_set_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_dsn_suppress_bit_is_set_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, bool cmock_to_return);
#define nrf_802154_frame_parser_dsn_suppress_bit_is_set_IgnoreArg_p_frame() nrf_802154_frame_parser_dsn_suppress_bit_is_set_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_dsn_suppress_bit_is_set_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_ie_present_bit_is_set_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_ie_present_bit_is_set_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_ie_present_bit_is_set_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, bool cmock_to_return);
#define nrf_802154_frame_parser_ie_present_bit_is_set_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_ie_present_bit_is_set_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_ie_present_bit_is_set_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, bool cmock_to_return);
typedef bool (* CMOCK_nrf_802154_frame_parser_ie_present_bit_is_set_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_ie_present_bit_is_set_StubWithCallback(CMOCK_nrf_802154_frame_parser_ie_present_bit_is_set_CALLBACK Callback);
#define nrf_802154_frame_parser_ie_present_bit_is_set_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_ie_present_bit_is_set_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_ie_present_bit_is_set_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, bool cmock_to_return);
#define nrf_802154_frame_parser_ie_present_bit_is_set_IgnoreArg_p_frame() nrf_802154_frame_parser_ie_present_bit_is_set_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_ie_present_bit_is_set_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_ie_header_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_ie_header_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_ie_header_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_ie_header_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_ie_header_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_ie_header_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t* cmock_to_return);
typedef uint8_t* (* CMOCK_nrf_802154_frame_parser_ie_header_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_ie_header_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_ie_header_get_CALLBACK Callback);
#define nrf_802154_frame_parser_ie_header_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_ie_header_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_ie_header_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t* cmock_to_return);
#define nrf_802154_frame_parser_ie_header_get_IgnoreArg_p_frame() nrf_802154_frame_parser_ie_header_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_ie_header_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);
#define nrf_802154_frame_parser_ie_header_offset_get_IgnoreAndReturn(cmock_retval) nrf_802154_frame_parser_ie_header_offset_get_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void nrf_802154_frame_parser_ie_header_offset_get_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_ie_header_offset_get_ExpectAndReturn(p_frame, cmock_retval) nrf_802154_frame_parser_ie_header_offset_get_CMockExpectAndReturn(__LINE__, p_frame, cmock_retval)
void nrf_802154_frame_parser_ie_header_offset_get_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, const uint8_t* p_frame, uint8_t cmock_to_return);
typedef uint8_t (* CMOCK_nrf_802154_frame_parser_ie_header_offset_get_CALLBACK)(const uint8_t* p_frame, int cmock_num_calls);
void nrf_802154_frame_parser_ie_header_offset_get_StubWithCallback(CMOCK_nrf_802154_frame_parser_ie_header_offset_get_CALLBACK Callback);
#define nrf_802154_frame_parser_ie_header_offset_get_ExpectWithArrayAndReturn(p_frame, p_frame_Depth, cmock_retval) nrf_802154_frame_parser_ie_header_offset_get_CMockExpectWithArrayAndReturn(__LINE__, p_frame, p_frame_Depth, cmock_retval)
void nrf_802154_frame_parser_ie_header_offset_get_CMockExpectWithArrayAndReturn(UNITY_LINE_TYPE cmock_line, uint8_t* p_frame, int p_frame_Depth, uint8_t cmock_to_return);
#define nrf_802154_frame_parser_ie_header_offset_get_IgnoreArg_p_frame() nrf_802154_frame_parser_ie_header_offset_get_CMockIgnoreArg_p_frame(__LINE__)
void nrf_802154_frame_parser_ie_header_offset_get_CMockIgnoreArg_p_frame(UNITY_LINE_TYPE cmock_line);

#endif
