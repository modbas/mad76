/**
  * MODBAS
  * madcar
  * Parameters
  *
  * Copyright (C) 2019, Frank Traenkle, http://www.modbas.de
  */


#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif


namespace mbmad
{

CarParameters* CarParameters::singleton = nullptr;

static CarParameters dummy;

CarParameters::CarParameters() noexcept
{
  singleton = this;
}

}
