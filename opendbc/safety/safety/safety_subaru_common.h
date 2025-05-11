#pragma once

#include "safety_declarations.h"

extern bool subaru_stop_and_go;
bool subaru_stop_and_go = false;

void subaru_common_init() {
  const int SUBARU_PARAM_SP_STOP_AND_GO = 1;

  subaru_stop_and_go = GET_FLAG(current_safety_param_sp, SUBARU_PARAM_SP_STOP_AND_GO);
}
