#include "error_status.h"

Error_Type_t error_indicator;

void Add_Error(Error_Enum_t error_state)
{
  error_indicator.error_list[error_indicator.error_index] = error_state;
  error_indicator.error_index++;
  if(error_indicator.error_index > (N_MAX_ERROR - 1))
  {
    error_indicator.error_index = N_MAX_ERROR - 1;
  }
}

