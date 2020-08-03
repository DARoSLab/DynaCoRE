// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved
#include "Common.h"

GRBException::GRBException(int errcode)
{
  error = errcode;
  msg   = "";
}

GRBException::GRBException(string errmsg, int errcode)
{
  error = errcode;
  msg   = errmsg;
}

const string 
GRBException::getMessage() const 
{ 
  return msg; 
}

int 
GRBException::getErrorCode() const
{ 
  return error; 
}


