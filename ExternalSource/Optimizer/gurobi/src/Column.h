// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved
#ifndef _COLUMN_CPP_H_
#define _COLUMN_CPP_H_

class GRBColumn
{
  private:

    vector<double> coeffs;
    vector<GRBConstr> constrs;

  public:

    unsigned int size(void) const;
    GRBConstr getConstr(int i) const;
    double getCoeff(int i) const;

    void addTerm(double coeff, GRBConstr constr);
    void addTerms(const double* coeff, const GRBConstr* constr, int cnt);
    void remove(int i);
    bool remove(GRBConstr c);

    void clear();
};
#endif
