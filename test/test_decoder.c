//
// Created by Aero on 2/2/17.
//


#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>
#include <inttypes.h>
#include <vn300_msg_types.h>
#include <vn300_encoder.h>
#include <vn300_msg_int.h>


#include "theft/greatest.h"
#include "theft/theft.h"
#include "vn300_decoder.h"


typedef struct theft  theft_t;
typedef struct {
    int limit;
    int fails;
    int dots;
    uint32_t foo;
} test_env_t;



static bool get_time_seed(theft_seed *seed)
{
  struct timeval tv;
  if (-1 == gettimeofday(&tv, NULL)) { return false; }
  *seed = (theft_seed)((tv.tv_sec << 32) | tv.tv_usec);
  /* printf("seed is 0x%016llx\n", *seed); */
  return true;
}

// === Setup for decoder fuzzing


static vn300_msg_buf_wrap_t* alloc_empty_standard_msg()
{

  vn300_msg_buf_wrap_t* pWrap = malloc(sizeof(vn300_msg_buf_wrap_t));
  if (NULL == pWrap) {
    return NULL;
  }

  pWrap->buf = malloc(vn300_standard_message_length());
  if (pWrap->buf == NULL) {
    return NULL;
  }

  pWrap->len = vn300_standard_message_length();

  return pWrap;

}

static void* encoded_buf_alloc_cb(theft_t *t, theft_seed seed, void *env)
{
  (void)env;

  vn300_msg_buf_wrap_t* pWrap = alloc_empty_standard_msg();
  if (pWrap == NULL) { return THEFT_ERROR; }

  //standard header declaration
  vn_encode_standard_header_group_fields(pWrap->buf);

  //randomize content of payload
  uint8_t* pBuf = pWrap->buf + VN_HEADER_PAYLOAD_OFF;
  const uint32_t payloadLen = vn300_standard_payload_length();
  const uint32_t phraseLen = sizeof(uint64_t);
  for (uint32_t i = 0; i < payloadLen; i += phraseLen, pBuf += phraseLen) {
    uint64_t phrase = theft_random(t);
    memcpy(pBuf, &phrase, phraseLen);
  }

  //calculate correct CRC
  const uint8_t* pCrcDataStart = (pWrap->buf + 1); //skip SYNC byte
  const uint32_t kStdMsgLenMinusCRC = (vn300_standard_message_length() - VN_CRC_LEN);
  const uint32_t kCrcDataLen = kStdMsgLenMinusCRC - 1;
  uint16_t current_crc = vn_u16_crc(pCrcDataStart,kCrcDataLen);
  //insert CRC at end of buffer
  uint8_t* pCrcOut = pWrap->buf + kStdMsgLenMinusCRC;
  memcpy(pCrcOut, &current_crc, sizeof(uint16_t));

  return pWrap;
}

static void encoded_buf_free_cb(void *instance, void *env)
{
  (void)env;
  vn300_msg_buf_wrap_t* pWrap = (vn300_msg_buf_wrap_t*)instance;
  free(pWrap->buf);
  pWrap->buf = NULL;
  free(pWrap);
}

static theft_hash encoded_buf_hash_cb(void *instance, void *env)
{
  (void)env;
  vn300_msg_buf_wrap_t* pWrap = (vn300_msg_buf_wrap_t*)instance;

  if (NULL != pWrap->buf) {
    return theft_hash_onepass(pWrap->buf, vn300_standard_message_length());
  }

  return 0;

}


static void print_standard_msg_buf(FILE *f, vn300_msg_buf_wrap_t* pWrap)
{
  for (uint32_t i = 0; i < pWrap->len; i++) {
    fprintf(f, " %02x ", pWrap->buf[i]);
  }
}

static void encoded_buf_print_cb(FILE *f, void *instance, void *env)
{
  (void)env;
  vn300_msg_buf_wrap_t* pWrap = (vn300_msg_buf_wrap_t*)instance;
  fprintf(f,"encoded_buf 0x%x buf: 0x%x len: %u data: ", (uint32_t)pWrap, (uint32_t)(pWrap->buf), pWrap->len);
  print_standard_msg_buf(f, pWrap);
}


static theft_progress_callback_res
generic_progress_cb(struct theft_trial_info *info, void *env)
{
  test_env_t *te = (test_env_t *)env;
  if ((info->trial & 0xff) == 0) {
    printf(".");
    fflush(stdout);
    te->dots++;
    if (te->dots == 64) {
      printf("\n");
      te->dots = 0;
    }
  }

  if (info->status == THEFT_TRIAL_FAIL) {
    te->fails++;
  }

  if (te->fails > 10) {
    return THEFT_PROGRESS_HALT;
  }
  return THEFT_PROGRESS_CONTINUE;
}

static struct theft_type_info encoded_buf_info = {
    .alloc = encoded_buf_alloc_cb,
    .free = encoded_buf_free_cb,
    .hash = encoded_buf_hash_cb,
//    .shrink = rbuf_shrink_cb,
    .print = encoded_buf_print_cb,
};

static theft_trial_res
prop_decoder_should_not_get_stuck(void* input)
{
  vn300_msg_buf_wrap_t* pWrap = (vn300_msg_buf_wrap_t*)input;
  vn300_standard_msg_t decoded = {0};

  vn300_decode_res decode_res = decode_standard_msg(pWrap, &decoded);
  //regardless of validity of input, this method should always return
  if (VN300_DECODE_OK != decode_res) {
    return THEFT_TRIAL_FAIL;
  }

  return THEFT_TRIAL_PASS;
}

static void set_random_u64(theft_t* t, uint64_t* out)
{
  *out = theft_random(t);
}


static void set_random_vec4f(theft_t* t, vn_vec4f* out)
{
  for (uint8_t i = 0; i < 4; i++) {
    out->c[i] = (float )theft_random_double(t);
  }
}

static void set_random_vec3f(theft_t* t, vn_vec3f* out)
{
  for (uint8_t i = 0; i < 3; i++) {
    out->c[i] = (float )theft_random_double(t);
  }
}

static void set_random_vec3d(theft_t* t, vn_vec3d* out)
{
  for (uint8_t i = 0; i < 3; i++) {
    out->c[i] = theft_random_double(t);
  }
}

static void set_random_pos3(theft_t* t, vn_pos3_t* out)
{
  set_random_vec3d(t, out);
}

static void set_random_vel3(theft_t* t, vn_vel3_t* out)
{
  set_random_vec3f(t,out);
}

static void* vn300_standard_msg_alloc_cb(theft_t* t, theft_seed seed, void *env)
{
  (void)env;

  vn300_standard_msg_t* pMsg = malloc(sizeof(vn300_standard_msg_t));
  if (pMsg == NULL) { return THEFT_ERROR; }
  memset((void*)pMsg,0,sizeof(vn300_standard_msg_t));

  set_random_u64(t, &pMsg->gps_nanoseconds);
  set_random_vec3f(t, &pMsg->angular_rate);
  set_random_vec3f(t, &pMsg->euler_yaw_pitch_roll); //VN_ATT_YawPitchRoll
  set_random_vec4f(t, &pMsg->att_quaternion); //VN_ATT_Quaternion

  set_random_pos3(t, &pMsg->pos_ecef);
  set_random_pos3(t, &pMsg->pos_lla);
  set_random_vel3(t, &pMsg->vel_body);
  set_random_vel3(t, &pMsg->vel_ned);

  pMsg->pos_uncertainty = (vn_pos)theft_random_double(t);
  pMsg->vel_uncertainty = (vn_vel)theft_random_double(t);

  return pMsg;
}

static void vn300_standard_msg_free_cb(void *instance, void *env)
{
  (void)env;
  vn300_standard_msg_t* pMsg = (vn300_standard_msg_t*)instance;
  free(pMsg);
}

static theft_hash vn300_standard_msg_hash_cb(void *instance, void *env)
{
  (void)env;
  vn300_standard_msg_t* pMsg = (vn300_standard_msg_t*)instance;
  return theft_hash_onepass( (uint8_t *)pMsg, sizeof(*pMsg));
}

static void print_vn300_standard_msg(FILE *f, const vn300_standard_msg_t *msg)
{
  fprintf(f, "gps_nanoseconds: %" PRIu64 "\n", (uint64_t)msg->gps_nanoseconds);
  fprintf(f, "angular_rate: [%6.3f, %6.3f, %6.3f]\n", msg->angular_rate.c[0], msg->angular_rate.c[1],msg->angular_rate.c[2]);
  fprintf(f, "euler_yaw_pitch_roll: [%6.3f, %6.3f, %6.3f]\n", msg->euler_yaw_pitch_roll.c[0], msg->euler_yaw_pitch_roll.c[1],msg->euler_yaw_pitch_roll.c[2]);

  fprintf(f, "pos_lla: [%6.3f, %6.3f, %6.3f]\n", msg->pos_lla.c[0], msg->pos_lla.c[1],msg->pos_lla.c[2] );
  fprintf(f, "pos_ecef: [%6.3f, %6.3f, %6.3f]\n", msg->pos_ecef.c[0], msg->pos_ecef.c[1],msg->pos_ecef.c[2] );
  fprintf(f, "vel_body: [%6.3f, %6.3f, %6.3f]\n", msg->vel_body.c[0], msg->vel_body.c[1],msg->vel_body.c[2] );
  fprintf(f, "vel_ned: [%6.3f, %6.3f, %6.3f]\n", msg->vel_ned.c[0], msg->vel_ned.c[1],msg->vel_ned.c[2] );
  fprintf(f, "pos_uncertainty: %6.3f \n",msg->pos_uncertainty);
  fprintf(f, "vel_uncertainty: %6.3f \n",msg->vel_uncertainty);
}

static void vn300_standard_msg_print_cb(FILE *f, void *instance, void *env)
{
  (void)env;
  vn300_standard_msg_t* pMsg = (vn300_standard_msg_t*)instance;
  fprintf(f," 0x%x : {\n", (uint32_t )pMsg);
  print_vn300_standard_msg(f, pMsg);
  fprintf(f,"\n}\n");
}



static struct theft_type_info vn300_standard_msg_info = {
  .alloc = vn300_standard_msg_alloc_cb,
    .free = vn300_standard_msg_free_cb,
    .hash = vn300_standard_msg_hash_cb,
    .print = vn300_standard_msg_print_cb,
};



/**
 * Given valid input, decoder output should match encoded input
 * @param input
 * @return
 */
static theft_trial_res
prop_decoded_should_match_encoded(void* input)
{
  vn300_standard_msg_t* pOrig = (vn300_standard_msg_t*)input;
  vn300_msg_buf_wrap_t encodedBuf = {0};

  //encode the original message struct as a buffer
  vn300_encode_res encode_res = encode_standard_msg(pOrig, &encodedBuf);
  if (VN300_ENCODE_OK != encode_res) {
    return THEFT_TRIAL_FAIL;
  }

  //decode the encoded buffer into a decoded message struct
  vn300_standard_msg_t decodedMsg;
  if (VN300_DECODE_OK != decode_standard_msg(&encodedBuf, &decodedMsg)) {
    return THEFT_TRIAL_FAIL;
  }

  //compare the original message struct to the decoded message struct
  //note that this only works because the messages are packed
  if (0 != memcmp(pOrig, &decodedMsg, sizeof(decodedMsg)) ) {
     return THEFT_TRIAL_FAIL;
  }

  return THEFT_TRIAL_PASS;
}




TEST encoded_and_decoded_data_should_match(void) {
  test_env_t env = { .limit = 1 << 11 };

  theft_seed seed;
  if (!get_time_seed(&seed)) { FAIL(); }

  struct theft *t = theft_init(0);
  struct theft_cfg cfg = {
      .name = __func__,
      .fun = prop_decoded_should_match_encoded,
      .type_info = {  &vn300_standard_msg_info },
      .seed = seed,
      .trials = 100000,
      .env = &env,
      .progress_cb = generic_progress_cb,
  };

  theft_run_res res = theft_run(t, &cfg);
  theft_free(t);
  printf("\n");
  ASSERT_EQ(THEFT_RUN_PASS, res);
  PASS();
}


TEST decoder_should_not_get_stuck(void)
{
  test_env_t env = { .limit = 1 << 11 };

  theft_seed seed;
  if (!get_time_seed(&seed)) { FAIL(); }

  struct theft *t = theft_init(0);
  struct theft_cfg cfg = {
      .name = __func__,
      .fun = prop_decoder_should_not_get_stuck,
      .type_info = {  &encoded_buf_info },
      .seed = seed,
      .trials = 10000,
      .env = &env,
      .progress_cb = generic_progress_cb,

  };

  theft_run_res res = theft_run(t, &cfg);
  theft_free(t);
  printf("\n");
      ASSERT_EQ(THEFT_RUN_PASS, res);
      PASS();
}

TEST decoder_should_reject_null_input_pointer(void)
{
  vn300_msg_buf_wrap_t* in = NULL;
  vn300_standard_msg_t out = {0};

  ASSERT_EQ(VN300_DECODE_EMPTY_INPUT,  decode_standard_msg(in, &out ) );
  PASS();
}

TEST decoder_should_reject_empty_msg(void)
{
  vn300_msg_buf_wrap_t in = {0};
  vn300_standard_msg_t out = {0};

  ASSERT_EQ(VN300_DECODE_EMPTY_INPUT,  decode_standard_msg(&in, &out ) );
  PASS();
}

TEST decoder_should_reject_missing_sync_header(void)
{
  vn300_msg_buf_wrap_t in = {
      .buf = (uint8_t*)"XOXO", //intentionally wrong sync header
      .len = vn300_standard_message_length(), //incorrect but unused
  };
  vn300_standard_msg_t out = {0};

  ASSERT_EQ(VN300_DECODE_BAD_INPUT,  decode_standard_msg(&in, &out ) );
  PASS();
}

TEST verify_crc(void)
{
  //16-bit CRC-CCITT ("Xmodem")
  //verified with https://www.lammertbies.nl/comm/info/crc-calculation.html

  const uint8_t* kTestData0 = (uint8_t*)"";
  const uint32_t kTestData0Len = 0;
  uint16_t testData0_crc = vn_u16_crc(kTestData0, kTestData0Len);
  ASSERT_EQ(0, testData0_crc);

  //CRC value for "ROCK" is 0x08FF
  const uint8_t* kTestData1 = (uint8_t*)"ROCK";
  const uint32_t kTestData1Len = strlen((char*)kTestData1);
  uint16_t testData1_crc = vn_u16_crc(kTestData1, kTestData1Len);
  ASSERT_EQ(0x08FF, testData1_crc);

  //CRC value for the string "12345678" is 0x9015
  const uint8_t* kTestData3 = (uint8_t*)"12345678";
  const uint32_t kTestData3Len = strlen((char*)kTestData3);
  uint16_t testData3_crc = vn_u16_crc(kTestData3, kTestData3Len);
  ASSERT_EQ(0x9015, testData3_crc);

  PASS();
}

SUITE(decoding)
{
  RUN_TEST(verify_crc);
  RUN_TEST(decoder_should_reject_null_input_pointer);
  RUN_TEST(decoder_should_reject_empty_msg);
  RUN_TEST(decoder_should_reject_missing_sync_header);
  RUN_TEST(decoder_should_not_get_stuck);
  RUN_TEST(encoded_and_decoded_data_should_match);
}


// Add all the definitions that need to be in the test runner's main file.
GREATEST_MAIN_DEFS();

int main(int argc, char **argv)
{
  GREATEST_MAIN_BEGIN(); // command-line arguments, initialization.
  RUN_SUITE(decoding);
  GREATEST_MAIN_END(); // display results
}