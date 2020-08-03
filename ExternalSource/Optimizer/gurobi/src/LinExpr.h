// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved
#ifndef _LINEXPR_CPP_H_
#define _LINEXPR_CPP_H_

class GRBLinExpr: public GRBExpr
{
  private:

    double constant;
    vector<double> coeffs;
    vector<GRBVar> vars;
    void multAdd(double m, const GRBLinExpr& expr);

  public:

    GRBLinExpr(double constant=0.0);
    GRBLinExpr(GRBVar var, double coeff=1.0);

    friend class GRBQuadExpr;

    friend ostream& operator<<(ostream &stream, GRBLinExpr expr);
    friend GRBLinExpr operator+(const GRBLinExpr& x, const GRBLinExpr& y);
    friend GRBLinExpr operator+(const GRBLinExpr& x);
    friend GRBLinExpr operator+(GRBVar x, GRBVar y);
    friend GRBLinExpr operator+(GRBVar x, double a);
    friend GRBLinExpr operator+(double a, GRBVar x);
    friend GRBLinExpr operator-(const GRBLinExpr& x, const GRBLinExpr& y);
    friend GRBLinExpr operator-(const GRBLinExpr& x);
    friend GRBLinExpr operator-(GRBVar x);
    friend GRBLinExpr operator-(GRBVar x, GRBVar y);
    friend GRBLinExpr operator-(GRBVar x, double a);
    friend GRBLinExpr operator-(double a, GRBVar x);
    friend GRBLinExpr operator*(double a, GRBVar x);
    friend GRBLinExpr operator*(GRBVar x, double a);
    friend GRBLinExpr operator*(const GRBLinExpr& x, double a);
    friend GRBLinExpr operator*(double a, const GRBLinExpr& x);
    friend GRBLinExpr operator/(GRBVar x, double a);
    friend GRBLinExpr operator/(const GRBLinExpr& x, double a);

    unsigned int size(void) const;
    GRBVar getVar(int i) const;
    double getCoeff(int i) const;
    double getConstant() const;
    double getValue() const;

    void addTerms(const double* coeff, const GRBVar* var, int cnt);
    GRBLinExpr operator=(const GRBLinExpr& rhs);
    void operator+=(const GRBLinExpr& expr);
    void operator-=(const GRBLinExpr& expr);
    void operator*=(double mult);
    void operator/=(double a);
    GRBLinExpr operator+(const GRBLinExpr& rhs);
    GRBLinExpr operator-(const GRBLinExpr& rhs);
    void remove(int i);
    bool remove(GRBVar v);

    void clear();
};
#endif
