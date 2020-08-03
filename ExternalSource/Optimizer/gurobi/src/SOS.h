// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved
#ifndef _SOS_CPP_H_
#define _SOS_CPP_H_

class GRBSOSRep // private one
{
  private:
    GRBmodel*  Cmodel;
    int        num;
  public:
    friend class GRBSOS;
};

class GRBSOS
{
  private:

    GRBSOSRep* sosRep;

    GRBSOS (GRBmodel* xmodel, int sos);
    void setindex(int sos);
    int  getindex() const;
    void remove();

  public:

    friend class GRBModel;

    GRBSOS();
    int get(GRB_IntAttr attr) const;
};
#endif
