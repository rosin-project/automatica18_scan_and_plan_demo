// Copyright 2018 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <modbus/modbus.h>
#include <unistd.h>

int main(int argc, char** argv)
{
  modbus_t * ctx = modbus_new_tcp("127.0.0.1", 1502);
  modbus_set_debug(ctx, TRUE);
  int s = modbus_tcp_listen(ctx, 1);
  modbus_tcp_accept(ctx, &s);

  modbus_mapping_t *mapping = modbus_mapping_new(2, 0, 0, 0);
  int rc = 0;

  uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH] = {};

  do{
    rc = modbus_receive(ctx, query);
    if(rc > 0) rc = modbus_reply(ctx, query, rc, mapping);
  } while(rc > 0);

  modbus_mapping_free(mapping);
  close(s);
  modbus_free(ctx);
  return 0;
}
