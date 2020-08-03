// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved
#ifndef _EXCEPTION_H_
#define _EXCEPTION_H_

class GRBException
{
  private:

    string msg;
    int error;

  public:

    GRBException(int errcode = 0);
    GRBException(string errmsg, int errcode = 0);

    const string getMessage() const;
    int getErrorCode() const;
};
#endif
