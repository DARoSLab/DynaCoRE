// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved
#ifndef _TEMPCONSTR_CPP_H_
#define _TEMPCONSTR_CPP_H_

class GRBTempConstr
{
  private:

    GRBQuadExpr expr;
    char        sense;

  public:

    friend class GRBModel;
    friend class GRBCallback;
    friend GRBTempConstr operator<=(GRBQuadExpr x, GRBQuadExpr y);
    friend GRBTempConstr operator>=(GRBQuadExpr x, GRBQuadExpr y);
    friend GRBTempConstr operator==(GRBQuadExpr x, GRBQuadExpr y);
};
#endif
